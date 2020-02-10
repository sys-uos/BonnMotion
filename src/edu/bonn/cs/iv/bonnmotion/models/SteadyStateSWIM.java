/*******************************************************************************
 ** BonnMotion - a mobility scenario generation and analysis tool             **
 ** Copyright (C) 2002-2012 University of Bonn                                **
 ** Copyright (C) 2012-2020 University of Osnabrueck                          **
 **                                                                           **
 ** This program is free software; you can redistribute it and/or modify      **
 ** it under the terms of the GNU General Public License as published by      **
 ** the Free Software Foundation; either version 2 of the License, or         **
 ** (at your option) any later version.                                       **
 **                                                                           **
 ** This program is distributed in the hope that it will be useful,           **
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of            **
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             **
 ** GNU General Public License for more details.                              **
 **                                                                           **
 ** You should have received a copy of the GNU General Public License along   **
 ** with this program; if not, write to the Free Software Foundation, Inc.,   **
 ** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.               **
 *******************************************************************************/

package edu.bonn.cs.iv.bonnmotion.models;


import java.io.*;
import java.util.Comparator;
import java.util.Locale;
import java.util.PriorityQueue;

import edu.bonn.cs.iv.bonnmotion.MobileNode;
import edu.bonn.cs.iv.bonnmotion.ModuleInfo;
import edu.bonn.cs.iv.bonnmotion.Position;
import edu.bonn.cs.iv.bonnmotion.models.SWIM.Event.Type;


@SuppressWarnings("Duplicates")
public class SteadyStateSWIM extends SWIM {
    // this is the precision for the numeric calculations of the inverse waittime function
    private static final double WAITTIME_THRESHOLD = 0.001;
    // lower bound of the Waitingtime function (metric in seconds)
    private static final double WAITTIME_LB = 1.0;
    private static double[] steadyWaitTimes;
    private static Position[] steadyPositions;

    private static String outputName = "timestamp_swim";
    private static boolean isSwimTracePrintingOn = true;

    private static final double PRECOMPUTE_CHECK_INTERVAL = 60;     // metric in DAYS
    private static final double DISTANCE_CHECK_INTERVAL = 60;     // metric in DAYS
    private static double precomputeTimeThreshold = 8 * 365 * 24 * 60 * 60;    // metric in seconds
    private static double precomputeValueThreshold = 0.001;
    private static int precomputeValueThresholdCounter = 0;         // counter to check whether for example the last 5 evaluations were below the threshold
    private static boolean isPreComputeModeOn = false;
    private static boolean isPreComputationOver = false;
    private boolean firstIterationPrecompute = true;
    private static double[] digestTimes;
    private static int digestPointer;
    private static boolean isDistancePrintingOn = false;
    private double[] distances;
    private double[] lastDistances;
    private boolean firstIterationDistances = true;
    private static BufferedWriter distancesOut = null;
    private static boolean isPercentageOfContactsOn = false;
    private static BufferedWriter percentageOfContactsOut = null;
    private double[][] lastPercentageOfContacts;   // Limit value y

    private String seenVectorFilename = null;
    private String seenVectorInput = null;
    private boolean isSeenStatic = false;
    private boolean isSeenStaticAndMean = false;
    private int[][] inputSeen;
    private static BufferedWriter seenVectorOut = null;


    // Module info
    private static ModuleInfo info;

    // Initialize module info.
    static {
        info = new ModuleInfo("SteadyStateSWIM");
        info.description = "Application to construct mobility scenarios according to the Small World in Motion model";

        info.major = 2;
        info.minor = 1;
        info.revision = ModuleInfo.getSVNRevisionStringValue("$LastChangedRevision: 2830 $");

        info.contacts.add(ModuleInfo.BM_MAILINGLIST);
        info.authors.add("Jan-Hendrik Bolte");
        info.references.add("http://swim.di.uniroma1.it/files/SWIM-Simulator.tar.gz");
        info.references.add("http://swim.di.uniroma1.it/files/SWIM-Infocom09.pdf");
        info.affiliation = ModuleInfo.UOS_SYS;
    }

    public static ModuleInfo getInfo() {
        isSuperclass = true;
        return info;
    }

    public SteadyStateSWIM(int _nodes, double _x, double _y, double _duration, double _ignore, long _randomSeed, double _nodeRadius, double _cellDistanceWeight, double _nodeSpeedMultiplier, double _waitingTimeExponent, double _waitingTimeUpperBound) {
        super(_nodes, _x, _y, _duration, _ignore, _randomSeed, _nodeRadius, _cellDistanceWeight, _nodeSpeedMultiplier, _waitingTimeExponent, _waitingTimeUpperBound);

        generate();
    }

    public SteadyStateSWIM(String[] _args) {
        go(_args);
    }

    public void go(String _args[]) {
        super.go(_args);
        generate();
    }

    public void generate() {
        Locale.setDefault(Locale.ENGLISH);

        try {
            if (isSwimTracePrintingOn) {
                bw = new BufferedWriter(new FileWriter(new File(outputName + "_SwimTrace.txt")));
            }
            if (isPreComputeModeOn) {
                seenVectorOut = new BufferedWriter(new FileWriter(new File(outputName + "_seen.txt")));
            }

            // prepare BonnMotion variables
            preGeneration();

            if (seenVectorInput != null) {
                seenVector(seenVectorInput);
            }

            int nodeCount = this.nodeCount();
            this.cellLength = nodeRadius / Math.sqrt(2.0);
            this.cellCountPerSide = (int) (Math.ceil(1.0 / cellLength));
            this.cellCount = cellCountPerSide * cellCountPerSide;

            this.id = new int[nodeCount];
            this.pos = new Position[nodeCount];
            this.state = new State[nodeCount];
            this.posTime = new double[nodeCount];
            this.dest = new Position[nodeCount];
            this.speed = new double[nodeCount];
            this.waitTime = new double[nodeCount];
            this.home = new Position[nodeCount];
            this.currentCell = new int[nodeCount];
            this.destinationCell = new int[nodeCount];
            this.density = new double[nodeCount];
            this.cellWeights = new double[nodeCount][];
            this.number_of_nodes_seen = new int[nodeCount][];
            this.number_of_nodes_seen_last_visit = new int[nodeCount][];

            meetInPlace = new BooleanWrapper[nodeCount][];
            for (int i = 0; i < nodeCount; ++i) {
                meetInPlace[i] = new BooleanWrapper[nodeCount];
            }
            for (int i = 0; i < nodeCount; ++i) {
                for (int j = 0; j < nodeCount; ++j) {
                    meetInPlace[i][j] = new BooleanWrapper(false);
                    meetInPlace[j][i] = new BooleanWrapper(false);
                }
            }

            for (int i = 0; i < nodeCount; i++) {
                Position homePos;
                if (isSeenStatic) {
                    homePos = steadyPositions[i];
                } else {
                    double y = this.randomNextDouble();
                    double x = this.randomNextDouble();
                    homePos = new Position(x, y);
                }

                id[i] = i;
                pos[i] = homePos;
                state[i] = State.NEW;
                posTime[i] = 0.0;
                dest[i] = homePos;
                speed[i] = 0.0;
                waitTime[i] = 0.0;
                home[i] = homePos;
                currentCell[i] = this.getCellIndexFromPos(homePos);
                destinationCell[i] = currentCell[i];
                density[i] = Math.PI * nodeRadius * nodeRadius * nodeCount;
                cellWeights[i] = new double[cellCount];
                number_of_nodes_seen[i] = new int[cellCount];
                number_of_nodes_seen_last_visit[i] = new int[cellCount];

                for (int j = 0; j < cellWeights[i].length; j++) {
                    cellWeights[i][j] = 0.0;
                    number_of_nodes_seen[i][j] = 0;
                    number_of_nodes_seen_last_visit[i][j] = 0;

                    if (cellDistanceWeight == 0.0) {
                        number_of_nodes_seen[i][j] = 1;
                        number_of_nodes_seen_last_visit[i][j] = 1;
                    }
                }
                parameterData.nodes[i] = new MobileNode();
            }

            if (isPreComputeModeOn) {
                BufferedWriter bwPos = new BufferedWriter(new FileWriter(new File(outputName + "_initpos.txt")));
                bwPos.write(nodeCount + "\n");
                for (int i = 0; i < nodeCount; i++) {
                    bwPos.write(Double.toString(pos[i].x));
                    bwPos.write(",");
                    bwPos.write(Double.toString(pos[i].y));
                    bwPos.write("\n");
                }
                bwPos.close();
            }

            this.initNodes();

            Comparator<Event> comp = new Comparator<Event>() {
                @Override
                public int compare(Event o1, Event o2) {
                    // the event with greater time has lower priority
                    if (o1.time >= o2.time) {
                        return 1;
                    } else {
                        return -1;
                    }
                }
            };

            this.eventQueue = new PriorityQueue<>(101, comp);

            currentTime = 0;

            // check for initial contacts
            for (int i = 0; i < nodeCount; i++) {
                for (int j = i + 1; j < nodeCount; j++) {
                    if (circles(new Position(getPosition(i).x, getPosition(i).y), nodeRadius, new Position(getPosition(j).x, getPosition(j).y), nodeRadius)) {
                        eventQueue.add(new Event(Type.MEET, i, j, 0));
                    }
                }
            }

            // create initial events
            for (int i = 0; i < nodeCount; i++) {
                if (isSeenStatic) {
                    steadyWaitTimes[i] = getRandomSteadyWaitingTime(this.waitingTimeExponent, WAITTIME_LB, this.waitingTimeUpperBound);
                    if (steadyWaitTimes[i] == -1) {
                        eventQueue.add(new Event(Type.START_MOVING, i, -1, 0));
                    } else {
                        eventQueue.add(new Event(Type.START_WAITING, i, -1, 0));
                    }
                } else {
                    eventQueue.add(new Event(Type.START_WAITING, i, -1, 0));
                }

            }

            // initialize Precompute Checktimer
            if (isPreComputeModeOn) {
                // initialize Vectors of everything
                this.lastPercentageOfContacts = new double[nodeCount][cellCount];
                if (isPercentageOfContactsOn) {
                    percentageOfContactsOut = new BufferedWriter(new FileWriter(new File(outputName + "_percentageOfContacts.txt")));
                }

            }

            if (isDistancePrintingOn) {
                distances = new double[nodeCount];
                lastDistances = new double[nodeCount];
                distancesOut = new BufferedWriter(new FileWriter(new File(outputName + "_distances.txt")));
            }

            if (isDistancePrintingOn || isPreComputeModeOn) {
                // Create enough space until last check interval
                double checkIntervallSeconds = PRECOMPUTE_CHECK_INTERVAL * 24 * 60 * 60;
                double precomputeTime = Math.ceil(precomputeTimeThreshold / checkIntervallSeconds) * checkIntervallSeconds;
                // add one place for last value, add x places for values in between
                digestTimes = new double[(int) (precomputeTime / this.waitingTimeUpperBound)
                        + (precomputeTime % this.waitingTimeUpperBound != 0 ? 1 : 0) + 1];

                // create digest timings
                for (int i = 0; i < digestTimes.length; i++) {
                    digestTimes[i] = i * this.waitingTimeUpperBound;
                }
                digestTimes[digestTimes.length - 1] = precomputeTime;
                digestPointer = 0;
            }

            outerLoop:
            while (true) {
                if (eventQueue.size() == 0) {
                    break;
                }

                Event e = eventQueue.poll();
                currentTime = e.time;

                // Last resort...then the calculation MUST be over (after time specified with -T!)
                if ((isDistancePrintingOn || isPercentageOfContactsOn) && currentTime > digestTimes[digestTimes.length - 1]) {
                    break;
                }

                // This is the normal limit
                if (!isPreComputeModeOn && currentTime >= parameterData.duration) {
                    break;
                }

                while ((isPreComputeModeOn || isDistancePrintingOn) && digestPointer < digestTimes.length && currentTime >= digestTimes[digestPointer]) {

                    // check every "PRECOMPUTE_CHECK_INTERVAL" days (a bit complicated due to the waitingtimeupperbound-dependent digest intervals)
                    if (digestPointer % (int) (PRECOMPUTE_CHECK_INTERVAL / ((this.waitingTimeUpperBound / 60 / 60) / 24.0)) == 0) {

                        if (isPreComputeModeOn) {
                            if (isPrecomputeFinished()) {
                                if (!isPreComputationOver) {
                                    // print seen vector (if for the first time (isPreComputationOver says that))
                                    StringBuilder sb = new StringBuilder();

                                    sb.append(currentTime);
                                    sb.append("\n");

                                    for (int i = 0; i < this.number_of_nodes_seen.length; i++) {
                                        for (int j = 0; j < this.number_of_nodes_seen[i].length; j++) {
                                            sb.append(this.number_of_nodes_seen[i][j]);
                                            if (j != this.number_of_nodes_seen[i].length - 1) {
                                                sb.append(" ");
                                            }
                                        }
                                        sb.append("\n");
                                    }
                                    sb.append("\n");

                                    seenVectorOut.write(sb.toString());
                                    isPreComputationOver = true;
                                }


                                // this is due to the fact, that the working version should print more than the specified thresholds
                                if (!isPercentageOfContactsOn) {
                                    break outerLoop;
                                } // else ignore
                            }
                        }
                    }


                    // check every "DISTANCE_CHECK_INTERVAL" days (a bit complicated due to the waitingtimeupperbound-dependent digest intervals)
                    if (digestPointer % (int) (DISTANCE_CHECK_INTERVAL / ((this.waitingTimeUpperBound / 60 / 60) / 24.0)) == 0) {
                        if (isDistancePrintingOn) {
                            if (firstIterationDistances) {
                                firstIterationDistances = false;
                            } else {
                                // calculate difference of distances for every node and print all these differences in one line
                                StringBuilder sb = new StringBuilder();
                                for (int i = 0; i < distances.length; i++) {
                                    sb.append(Math.abs(distances[i] - lastDistances[i]));

                                    if (i == distances.length - 1) {
                                        sb.append("\n");
                                    } else {
                                        sb.append(",");
                                    }
                                }
                                distancesOut.write(sb.toString());
                                distancesOut.flush();
                            }

                            lastDistances = distances;
                            distances = new double[nodeCount];
                        }
                    }
                    System.out.printf("\r%7.2f%%", digestTimes[digestPointer] / digestTimes[digestTimes.length - 1] * 100);
                    digestPointer++;
                }

                if (currentTime > parameterData.ignore) {
                    if (e.type == Type.MEET) {
                        if ((getState(e.firstNode) == State.WAITING && getState(e.secondNode) == State.MOVING)) {
                            if (circles(getPosition(e.firstNode), nodeRadius, getDestination(e.secondNode), nodeRadius)) {
                                meetInPlace[e.firstNode][e.secondNode].val = true;
                                meetInPlace[e.secondNode][e.firstNode].val = true;
                                PrintEvent(e, "MP");
                            } else {
                                PrintEvent(e, "MM");
                            }
                        } else {
                            if ((getState(e.firstNode) == State.MOVING && getState(e.secondNode) == State.MOVING)) {
                                if (circles(getDestination(e.firstNode), nodeRadius, getDestination(e.secondNode), nodeRadius)) {
                                    // Find the time in which at least one node
                                    // reaches the destinations
                                    double minTravelTime = Math.min(getPositionAbsoluteTime(e.firstNode) + getTravelTime(e.firstNode), getPositionAbsoluteTime(e.secondNode) + getTravelTime(e.secondNode));

                                    // If in that time the nodes are still seeing
                                    // each other we print MP, otherwise we print MM
                                    if (circles(computePositionAtTime(minTravelTime, e.firstNode), nodeRadius, computePositionAtTime(minTravelTime, e.secondNode), nodeRadius)) {
                                        meetInPlace[e.firstNode][e.secondNode].val = true;
                                        meetInPlace[e.secondNode][e.firstNode].val = true;
                                        PrintEvent(e, "MP");
                                    } else {
                                        PrintEvent(e, "MM");
                                    }
                                } else {
                                    PrintEvent(e, "MM");
                                }
                            } else if ((getState(e.firstNode) == State.MOVING && getState(e.secondNode) == State.WAITING)) {
                                if (circles(getDestination(e.firstNode), nodeRadius, getPosition(e.secondNode), nodeRadius)) {
                                    meetInPlace[e.firstNode][e.secondNode].val = true;
                                    meetInPlace[e.secondNode][e.firstNode].val = true;
                                    PrintEvent(e, "MP");
                                } else {
                                    PrintEvent(e, "MM");
                                }
                            }
                        }
                    } else if (e.type == Type.LEAVE) {
                        if (meetInPlace[e.firstNode][e.secondNode].val || meetInPlace[e.secondNode][e.firstNode].val) {
                            meetInPlace[e.firstNode][e.secondNode].val = false;
                            meetInPlace[e.secondNode][e.firstNode].val = false;
                            PrintEvent(e, "LP");
                        } else {
                            PrintEvent(e, "LM");
                        }
                    } else {
                        PrintEvent(e);
                    }
                }

                // handle event
                switch (e.type) {
                    case START_MOVING:
                        updatePosition(e.firstNode, getDestination(e.firstNode), currentTime);
                        moveToRandomDestination(e.firstNode);
                        eventQueue.add(new Event(Type.END_MOVING, e.firstNode, -1, e.time + getTravelTime(e.firstNode)));
                        checkContacts(e.firstNode);
                        break;

                    case START_WAITING:

                        // calc distance and add it to the sum of distances
                        if (isDistancePrintingOn) {
                            Position from = pos[e.firstNode];
                            Position to = getDestination(e.firstNode);
                            double dist = Math.sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));
                            distances[e.firstNode] += dist;
                        }
                        updatePosition(e.firstNode, getDestination(e.firstNode), currentTime);
                        waitRandomTime(e.firstNode);
                        double timeToWait = getTravelTime(e.firstNode);
                        eventQueue.add(new Event(Type.END_WAITING, e.firstNode, -1, e.time + timeToWait));
                        checkContacts(e.firstNode);
                        break;

                    case END_MOVING:
                        eventQueue.add(new Event(Type.START_WAITING, e.firstNode, -1, e.time));
                        break;

                    case END_WAITING:
                        eventQueue.add(new Event(Type.START_MOVING, e.firstNode, -1, e.time));
                        break;

                    case MEET:
                        if (!isSeenStatic) {
                            meet(e.firstNode, e.secondNode);
                            meet(e.secondNode, e.firstNode);
                        }
                        break;

                    case LEAVE:
                        break;

                    default:
                        break;
                }
            }
            System.out.println();
            postGeneration();

            // During preComputation no movement is saved for movements.gz, but there must be at least one.
            // Because of that add one movement for every node
            if (isPreComputeModeOn) {
                for (int i = 0; i < parameterData.nodes.length; i++) {
                    parameterData.nodes[i].add(0, new Position(0, 0));
                }
            }

            if (isSwimTracePrintingOn) {
                bw.flush();
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                if (isSwimTracePrintingOn) {
                    bw.close();
                }
                if (isPreComputeModeOn) {
                    seenVectorOut.close();
                    if (isPercentageOfContactsOn) {
                        percentageOfContactsOut.close();
                    }
                }
                if (isDistancePrintingOn) {
                    distancesOut.close();
                }
            } catch (IOException e) {
                System.err.println("Files couldn't be closed.");
                e.printStackTrace();
            }
        }
    }


    /**
     * method that says whether precompute phase is done.
     *
     * @return true if yes, false if no
     */
    private boolean isPrecomputeFinished() {
        double currentPercentageOfContacts;
        double diff;
        double maxDiff = 0.0;
        int seen;
        int allSeens;
        String sep;

        for (int i = 0; i < this.parameterData.nodes.length; i++) {
            for (int j = 0; j < this.cellCount; j++) {

                seen = this.number_of_nodes_seen[i][j];

                // create y value
                // gather all seens of node everywhere
                allSeens = 0;
                for (int k = 0; k < number_of_nodes_seen[i].length; k++) {
                    allSeens += number_of_nodes_seen[i][k];
                }

                currentPercentageOfContacts = (allSeens == 0.0 ? 0.0 : (double) seen / allSeens);
                if (!firstIterationPrecompute) {
                    diff = currentPercentageOfContacts - lastPercentageOfContacts[i][j];

                    // determine maximum difference in order to get the threshold value
                    if (diff > maxDiff) {
                        maxDiff = diff;
                    }

                    // print diff to debug file
                    if (isPercentageOfContactsOn) {
                        try {
                            percentageOfContactsOut.write(diff + "");
                        } catch (IOException e) {
                            System.err.println("Error while writing file.");
                            e.printStackTrace();
                        }
                    }

                }

                // set new percentage of contacts to variable
                this.lastPercentageOfContacts[i][j] = currentPercentageOfContacts;

                // print correct separator to debug file
                if (!firstIterationPrecompute && isPercentageOfContactsOn) {
                    if (i == this.parameterData.nodes.length - 1 && j == this.cellCount - 1) {
                        sep = "\n";
                    } else {
                        sep = ",";
                    }

                    try {
                        percentageOfContactsOut.write(sep);
                    } catch (IOException e) {
                        System.err.println("Error while writing file.");
                        e.printStackTrace();
                    }
                }


            }
        }

        if (firstIterationPrecompute) {
            firstIterationPrecompute = false;
            precomputeValueThresholdCounter = 0;
        } else {
            // check whether max value is lower than the threshold
            if (maxDiff < precomputeValueThreshold) {
                precomputeValueThresholdCounter++;
                // Check if the last 5 evaluations exceeded the threshold. This makes it more robust against "accidental" minima.
                if (precomputeValueThresholdCounter == 5) {
                    System.out.println("\nNote: Precomputation ended because the VALUE-threshold (" + precomputeValueThreshold + ") was exceeded in the last 5 evaluations.");
                    return true;
                }
            } else {
                precomputeValueThresholdCounter = 0;
            }
            // check whether time threshold is exceeded
            if (digestTimes[digestPointer] > precomputeTimeThreshold) {
                System.out.println("\nNote: Precomputation ended because the TIME-threshold (" + precomputeTimeThreshold / 60 / 60 / 24 / 365 + " years) was exceeded in the last evaluation.");
                return true;
            }
        }

        return false;
    }


    private int[] getMeanVector(int[][] vector) {
        int[] meanVector = new int[vector[0].length];
        for (int i = 0; i < vector.length; i++) {
            for (int j = 0; j < vector[0].length; j++) {
                meanVector[j] += vector[i][j];
            }
        }
        // normalize on number of nodes
        for (int i = 0; i < vector[0].length; i++) {
            meanVector[i] /= vector.length;
        }
        return meanVector;
    }


    protected boolean parseArg(char _key, String _value) {
        switch (_key) {
            case 'E':
                // PrecomputeMode
                isPreComputeModeOn = true;
                isSwimTracePrintingOn = false;
                return true;
            case 'N':
                // PercentageOfContacts Mode
                isPercentageOfContactsOn = true;
                isPreComputeModeOn = true;
                isSwimTracePrintingOn = false;
                return true;
            case 'T': // set precompute time threshold
                precomputeTimeThreshold = Double.parseDouble(_value);
                return true;
            case 'V': // set precompute value threshold
                precomputeValueThreshold = Double.parseDouble(_value);
                return true;
            case 'D':
                // Print distance measurement
                isDistancePrintingOn = true;
                return true;
            case 'S': // prohibit SWIMTRACE printing
                isSwimTracePrintingOn = false;
                return true;
            case 'K': // for providing a seen vector
                // prefix of filename, timeofvector
                seenVector(_value);
                return true;
            case 'M': // for only take the mean seen vector for every node.
                isSeenStaticAndMean = true;
                return true;
            default:
                return super.parseArg(_key, _value);
        }
    }

    protected boolean parseArg(String _key, String _value) {
        switch (_key) {
            case "precomputeMode":
                // PrecomputeMode
                if (Boolean.parseBoolean(_value)) {
                    isPreComputeModeOn = true;
                    isSwimTracePrintingOn = false;
                }
                return true;
            case "percentageOfContactsMode":
                // PercentageOfContacts Mode
                if (Boolean.parseBoolean(_value)) {
                    isPercentageOfContactsOn = true;
                    isPreComputeModeOn = true;
                    isSwimTracePrintingOn = false;
                }
                return true;
            case "timeThreshold": // set precompute time threshold
                precomputeTimeThreshold = Double.parseDouble(_value);
                return true;
            case "valueThreshold": // set precompute value threshold
                precomputeValueThreshold = Double.parseDouble(_value);
                return true;
            case "distancePrinting":
                // Print distance measurement
                isDistancePrintingOn = Boolean.parseBoolean(_value);
                return true;
            case "prohibitSwimTracePrinting": // prohibit SWIMTRACE printing
                isSwimTracePrintingOn = !Boolean.parseBoolean(_value);
                return true;
            case "seenVector": // for providing a seen vector
                // prefix of filename, timeofvector
                seenVectorInput = _value;
                return true;
            case "meanSeenVector": // for only take the mean seen vector for every node.
                isSeenStaticAndMean = Boolean.parseBoolean(_value);
                return true;
            default:
                return super.parseArg(_key, _value);
        }
    }

    //			--> Maybe think about packing precomputation in one single call of the model in order to achieve better user feeling
    public void write(String _filename) throws FileNotFoundException, IOException {
        // Change temporary filenames ("timestamp_swim") to the right filename according to _filename
        File[] files = new File(".").listFiles();
        if (files != null) {
            for (File value : files) {
                if (value.isFile() && value.getName().contains(outputName)) {
                    String[] name = value.getName().split(outputName);

                    // delete existing old files with the new filename, otherwise windows couldn't rename the file
                    File tmp = new File(_filename + name[1]);
                    tmp.delete();

                    if (!value.renameTo(new File(_filename + name[1]))) {
                        System.err.println("Parameter files couldn't be renamed.");
                    }
                }
            }
        }

        String[] p = new String[8];
        int i = 0;
        // Steady State parameter
        p[i] = "precomputeMode=";
        p[i++] += isPreComputeModeOn && !isSwimTracePrintingOn && !isPercentageOfContactsOn;
        p[i++] = "percentageOfContactsMode=" + isPercentageOfContactsOn;
        p[i++] = "timeThreshold=" + precomputeTimeThreshold;
        p[i++] = "valueThreshold=" + precomputeValueThreshold;
        p[i++] = "distancePrinting=" + isDistancePrintingOn;
        p[i++] = "prohibitSwimTracePrinting=" + !isSwimTracePrintingOn;
        if (seenVectorFilename != null)
            p[i++] = "seenVector=" + seenVectorFilename;
        p[i++] = "meanSeenVector=" + isSeenStaticAndMean;

        // Shrink array if not every parameter is used
        if (i != p.length) {
            String[] pNew = new String[i];
            System.arraycopy(p, 0, pNew, 0, i);
            paramsSubclass = pNew;
            super.write(_filename);
            return;
        }
        paramsSubclass = p;
        super.write(_filename);
    }

    public static void printHelp() {
        System.out.println(getInfo().toDetailString());
        SWIM.printHelp();
        System.out.println(getInfo().name + ":");
        System.out.println("\t-E                                                     Precompute Mode on (no argument)\n" +
                "\t-N                                                     PercentageofContacts Mode on (no argument). Is like -E but with debug of metric PercentageOfContacts. Value threshold will be ignored.\n" +
                "\t-T <time threshold in s>                               The precomputation is over when the time threshold was exceeded for the first time.\n" +
                "\t-V <value threshold>                                   The precomputation is over when the value threshold (percentage of contacts) was exceeded the last 5 times.\n" +
                "\t-D                                                     DistancePrinting Mode on (no argument).\n" +
                "\t-S                                                     prohibit creation of Swimtrace (used for contact metrics) (no argument)\n" +
                "\t-K <prefix filename of seen vector and initpos>        for static mode with raw vector. A <prefix filename>_seen.txt and <prefix filename>_initpos.txt file must exist!\n" +
                "\t-M                                                     Apply mean to static seen vector. -K needs to be set in addition to this!\n"
        );
    }

    /**
     * initialize seen Vector if seen Vector Argument is set
     *
     * @param _value prefix of filename, timeofvector
     */
    private void seenVector(String _value) {
        isSeenStatic = true;
        seenVectorFilename = _value.split(",")[0];
        // int timestamp = Integer.parseInt(_value.split(",")[1]);

        // count lines
        int lineCnt = 0;
        int tooMuch = 0;
        try (BufferedReader br = new BufferedReader(new FileReader(seenVectorFilename + "_seen.txt"))) {
            while (br.readLine() != null) {
                lineCnt++;
                for (int i = 0; i < this.parameterData.nodes.length; i++) {
                    if (br.readLine() == null) {
                        tooMuch++;
                        break;
                    }
                }
                br.readLine();
            }
        } catch (FileNotFoundException e) {
            System.err.println("Couldn't find seen Vector file: " + seenVectorFilename + "_seen.txt");
            e.printStackTrace();
            System.exit(-1);
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(-1);
        }

        System.out.printf("Dropped %d from %d lines\n", tooMuch, lineCnt);

        lineCnt -= tooMuch;

        try (BufferedReader br = new BufferedReader(new FileReader(seenVectorFilename + "_seen.txt"))) {
            String input = null;
            String[] line;
            int cnt = 0;
            while (true) {
                input = br.readLine();
                if (input == null) {
                    break;
                }
                cnt++;
                int time = (int) Double.parseDouble(input);
                if (cnt == lineCnt) {
                    System.out.printf("Seen taken from Time: %d\n", time);
                    // read Vector
                    for (int nodeID = 0; nodeID < this.parameterData.nodes.length; nodeID++) {
                        line = br.readLine().split(" ");
                        if (inputSeen == null) {
                            inputSeen = new int[this.parameterData.nodes.length][line.length];
                        }
                        for (int cellID = 0; cellID < line.length; cellID++) {
                            inputSeen[nodeID][cellID] = Integer.parseInt(line[cellID]);
                            if (cellID == line.length - 1) {
                            } else {
                            }

                        }
                    }
                } else {
                    // überspringe
                    for (int i = 0; i < this.parameterData.nodes.length; i++) {
                        br.readLine();
                    }
                }
                br.readLine();
            }
        } catch (FileNotFoundException e) {
            System.err.println("Couldn't find seen Vector file: " + seenVectorFilename + "_seen.txt");
            e.printStackTrace();
            System.exit(-1);
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(-1);
        }


        /* READ IN THE INITIAL POSITIONS */
        try (BufferedReader br = new BufferedReader(new FileReader(seenVectorFilename + "_initpos.txt"))) {
            int nodes = (int) Double.parseDouble(br.readLine());
            steadyPositions = new Position[nodes];
            String input;
            String[] coords;
            for (int i = 0; i < nodes; i++) {
                input = br.readLine();
                coords = input.split((","));
                steadyPositions[i] = new Position(Double.parseDouble(coords[0]), Double.parseDouble(coords[1]));
            }
        } catch (FileNotFoundException e) {
            System.err.println("Couldn't find initpos file: " + seenVectorFilename + "_initpos.txt");
            e.printStackTrace();
            System.exit(-1);
        } catch (IOException e) {
            System.err.println("Error while reading file.");
            e.printStackTrace();
            System.exit(-1);
        }
    }

    public void initNodes() {
        if (isSeenStaticAndMean) {
            if (!isSeenStatic) {
                System.out.println("[!] Error: A seen vector needs to be provided for option -M!");
                System.exit(-1);
            }

            // Calculate mean seen vector from seen vector already provided
            int[] meanSeen = getMeanVector(inputSeen);

            // write to arrays
            for (int i = 0; i < inputSeen.length; i++) {
                for (int j = 0; j < meanSeen.length; j++) {
                    inputSeen[i][j] = meanSeen[j];
                }
            }
        }

        if (isSeenStatic) {
            number_of_nodes_seen = inputSeen;
            steadyWaitTimes = new double[parameterData.nodes.length];
        }

        for (int i = 0; i < parameterData.nodes.length; i++) {
            initCellWeights(i);
        }
    }

    public void moveToRandomDestination(int _index) {
        if (!isSeenStatic) {
            setCellWeight(_index, currentCell[_index], number_of_nodes_seen_last_visit[_index][currentCell[_index]]);
            number_of_nodes_seen_last_visit[_index][currentCell[_index]] = 0;
        }

        destinationCell[_index] = chooseDestinationCell(_index);

        Position destinationPoint = this.getRandomPointInCell(destinationCell[_index]);

        state[_index] = State.MOVING;
        dest[_index] = destinationPoint;
        speed[_index] = (destinationPoint.newShiftedPosition(-pos[_index].x, -pos[_index].y)).norm() * nodeSpeedMultiplier;
        waitTime[_index] = 0.0;
    }

    public void waitRandomTime(int _index) {
        double waittime;
        if (isSeenStatic && steadyWaitTimes[_index] != -1.0) {
            waittime = steadyWaitTimes[_index];
            steadyWaitTimes[_index] = -1.0;
        } else {
            waittime = computeRandomWaitingTime();
        }
        state[_index] = State.WAITING;
        dest[_index] = pos[_index];
        speed[_index] = 0.0;
        waitTime[_index] = waittime;
        currentCell[_index] = destinationCell[_index];
    }


    private double getRandomSteadyWaitingTime(double b, double p, double c) {

        double what = this.randomNextDouble();

        double left = p;
        double right = c;
        double mitte = (left + right) / 2;

        if (what <= steadyWaitingTime(b, p, c, p)) {
            return -1;
        }

        double ret = steadyWaitingTime(b, p, c, mitte);

        while (Math.abs(ret - what) > WAITTIME_THRESHOLD) {

            if (what > ret) {
                left = mitte;
            }

            if (what < ret) {
                right = mitte;
            }

            mitte = (left + right) / 2;
            ret = steadyWaitingTime(b, p, c, mitte);
        }
        return mitte;
    }


    /**
     * Method for getting a waiting time distribution in the beginning when
     * working with a fixed seen vector.
     *
     * @param b Beta value (steepness of waiting time function)
     * @param p Minimum value for waiting time
     * @param c Maximum value for waiting time
     * @param x Value for which the waiting time probability should be given
     * @return A random Waiting time
     */
    private double steadyWaitingTime(double b, double p, double c, double x) {

        assert p <= x;
        assert x <= c;
        assert b > 1;

        double h1 = b / (b - 1);
        double h2 = pow(p, b);
        double h3 = p / c;
        double linksOben, linksUnten, rechtsOben, rechtsUnten;


        linksOben = h1 * p - h2 / (b - 1) * pow(x, 1.0 - b) - pow(h3, b) * x;
        linksUnten = h2 * h1 + 1 - 1 / (b - 1) * pow(h3, b) * (b * c + b - 1);
        rechtsOben = 1;
        rechtsUnten = h2 / (1 - pow(h3, b)) * h1 * (1 / (pow(p, b - 1)) - 1 / (pow(c, b - 1))) + 1;

        double links = linksOben / linksUnten;
        double rechts = rechtsOben / rechtsUnten;

        return links + rechts;
    }

    public double pow(double a, double b) {
        return Math.pow(a, b);
    }

    public void PrintEvent(Event e, String eventType) throws IOException {
        if (isSwimTracePrintingOn) {
            String go = String.format("%.3f %s %4d %4d %.3f %.3f %.3f %.3f\n", e.time - parameterData.ignore, eventType, e.firstNode, e.secondNode, computePositionAtTime(currentTime, e.firstNode).x, computePositionAtTime(currentTime, e.firstNode).y, computePositionAtTime(currentTime, e.secondNode).x, computePositionAtTime(currentTime, e.secondNode).y);
            bw.write(go);
        }
    }

    public void PrintEvent(Event e) throws IOException {
        String eventType = "NOT_INITIALIZED";
        switch (e.type) {
            case START_MOVING: {
                eventType = "SM";
                break;
            }

            case START_WAITING: {
                eventType = "SW";
                break;
            }

            case END_MOVING: {
                eventType = "EM";
                break;
            }

            case END_WAITING: {
                eventType = "EW";
                break;
            }
            default:
                break;
        }

        if (eventType.equals("SM") || eventType.equals("EM")) {
            Position curPos = computePositionAtTime(e.time, e.firstNode);

            // Don't add the movements during preComputation, because it needs to much RAM and preComputation movements in movements.gz aren't necessary
            if (!isPreComputeModeOn) {
                parameterData.nodes[e.firstNode].add(e.time, new Position(curPos.x * this.parameterData.x, curPos.y * this.parameterData.y));
            }
        }
        if (isSwimTracePrintingOn) {
            String go = String.format("%.3f %s %4d %.3f %.3f\n", e.time - parameterData.ignore, eventType, e.firstNode, computePositionAtTime(currentTime, e.firstNode).x, computePositionAtTime(currentTime, e.firstNode).y);
            bw.write(go);
        }
    }
}