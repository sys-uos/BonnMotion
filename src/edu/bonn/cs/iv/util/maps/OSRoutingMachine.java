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

package edu.bonn.cs.iv.util.maps;

import java.awt.geom.Point2D;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.URL;
import java.net.HttpURLConnection;
import java.net.URLConnection;
import java.text.DecimalFormat;
import java.util.Iterator;
import java.util.Vector;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.JsonNode;

/*
* @see http://project-osrm.org/
*/
public class OSRoutingMachine implements RouteServiceInterface
{	
    private static final boolean DEBUG = true;
    
    /** threshold for accepting a NO_ROUTE response as a flight */
    public static final int ROUTE_AS_FLIGHT_THRESH = 0;
    
    private static final String OSRM_URL = "http://192.168.95.134:5000"; // shortest
    private String serviceUrl = null;
    public PrintWriter distWriter = null;
    
    public OSRoutingMachine(String serviceUrl, String distFile)
    {
        this.serviceUrl = serviceUrl;
        
        if (serviceUrl == null) {
            this.serviceUrl = OSRM_URL;
        }
        
        if (distFile != null) {
            try {
                this.distWriter = new PrintWriter(new FileOutputStream(distFile));
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public Route getOptimalRoute(PositionGeo src, PositionGeo dst) throws RSIRequestFailedException
    {
    
        Route result = new Route(src, dst);
        
        if (almostEqual(src, dst)) { // src and dst are very close; might result in failure anyway
            result.setTurningPoint(new PositionGeo[0]);
            return result;
        }
                
        
        String request = String.format("%s/route/v1/driving/%s,%s;%s,%s?generate_hints=false&geometries=geojson&overview=full", serviceUrl, src.lon(), src.lat(), dst.lon(), dst.lat());
        
        result.setRequest(request);
        
        //System.out.println("Route request: " + request);

        try {
            // send locate request
            URL url = new URL(request);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setReadTimeout(30000);
            conn.setConnectTimeout(30000);
            conn.connect();

            if (conn.getResponseCode() == 200) {
                // receive and parse JSON-formatted response
                ObjectMapper om = new ObjectMapper();
                JsonNode root = om.readTree(conn.getInputStream());
                
                String status =  root.get("code").asText();                
                
                try {
                        processResponseStatus(status);
                    } catch (RSIRequestFailedException e) {
                        System.err.println(request);
                        
                        if (status == "NoRoute" && almostEqual(getNearestPosition(src), getNearestPosition(dst))) { // OSRM maps src and dst to the same position
                            result.setTurningPoint(new PositionGeo[0]);
                            return result;
                        } else if (status == "NoRoute" && src.distance(dst) < ROUTE_AS_FLIGHT_THRESH) {
                            result.setTurningPoint(new PositionGeo[0]);
                            return result;
                        } else {
                            System.err.println(e.getMessage());
                            e.printStackTrace();
                            throw e;
                        }
                    }
                
                JsonNode route = root.get("routes").elements().next();
                result.setDistanceRS(route.get("distance").asDouble());
                result.setTripTime(route.get("duration").asDouble());
                    
                Vector<PositionGeo> tpLonLat = new Vector<PositionGeo>();
                Iterator<JsonNode> coordinates = route.get("geometry").get("coordinates").elements();
                while(coordinates.hasNext()) {
                    Iterator<JsonNode> locs = coordinates.next().elements();
                    double lon = locs.next().asDouble();
                    double lat = locs.next().asDouble();
                    tpLonLat.add(new PositionGeo(lon, lat));
                }
                
                result.setStartPoint(tpLonLat.firstElement().toString());
                result.setEndPoint(tpLonLat.get(tpLonLat.size()-1).toString());

                // check whether tpLonLat contains src and dst                
                if (almostEqual(src, tpLonLat.firstElement())) {
                    tpLonLat.remove(0);
                }
                if (almostEqual(dst, tpLonLat.lastElement())) {
                    tpLonLat.remove(tpLonLat.size() - 1);
                }
                
                result.setTurningPoint(tpLonLat.toArray(new PositionGeo[tpLonLat.size()]));
                
            
            } else {
                throw new RSIRequestFailedException("OSRM Request unsuccessful (http request error)");		
            }
        } catch (Exception e) {
            System.err.println(e.toString());
            e.printStackTrace();
            if (e instanceof RSIRequestFailedException) {
                throw (RSIRequestFailedException)e;
            } else {
                System.exit(-1);
            }
        }
//        if (DEBUG) System.out.println("result = " + result.toString());

        if (distWriter != null) {
            distWriter.printf("%.6f %.6f %.6f %.6f %.2f %.2f %.4f\n", result.src().x(), result.src().y(), result.dst().x(), result.dst().y(), result.flightLength(), result.distanceGeodesic(), result.distanceGeodesic() / result.flightLength());
        }
        
        return result;
    }
    
    @Override
    public boolean isRoutable(PositionGeo p)
    {
        PositionGeo np = null;
        try {
            np = getNearestPosition(p);
        } catch (RSIRequestFailedException e) {
            System.err.println(e.getMessage());
            e.printStackTrace();
            System.exit(-1);
        }
        return np.equals(p);
    }

    @Override
    public PositionGeo getNearestPosition(PositionGeo p) throws RSIRequestFailedException
    {
        PositionGeo result = null;

        String request = String.format("%s/nearest/v1/driving/%s,%s?number=1&generate_hints=false", serviceUrl, p.lon(), p.lat());
                
//        if (DEBUG) System.out.println("OSRM Request: "+request);
        
        // System.out.println("Nearest request: " + request);

        try {
            // send locate request
            URL url = new URL(request);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setReadTimeout(30000);
            conn.setConnectTimeout(30000);
            conn.connect();

            if (conn.getResponseCode() == 200) {
                // receive and parse JSON-formatted response
                ObjectMapper om = new ObjectMapper();
                JsonNode root = om.readTree(conn.getInputStream());
                
                processResponseStatus(root.get("code").asText());
                
                //System.out.println(root);
                //System.out.println(root.get("code").asText());
                Iterator<JsonNode> locs = root.get("waypoints").elements().next().get("location").elements();
                //System.out.println(locs);
                double lon = locs.next().asDouble();
                double lat = locs.next().asDouble();
           
                result = new PositionGeo(lon, lat);
            
            } else {
                throw new RSIRequestFailedException("OSRM Request unsuccessful (http request error)");		
            }
                        
//            if (DEBUG) System.out.println("NearestPosition():\nquery: "+this.toString(posLonLat)+"\nresponse: "+this.toString(resultLonLat));
            
//            if (almostEqual(p, result)) {
//            	result = p;
//            }
        } catch (Exception e) {
            System.err.println(e.toString());
            e.printStackTrace();
            if (e instanceof RSIRequestFailedException) {
                throw (RSIRequestFailedException)e;
            } else {
                System.exit(-1);
            }
        }
//        if (DEBUG) System.out.println("result = " + result.toString());

        return result;
    }

    
    /*
    * @param p1 Lon/Lat position
    * @param p2 Lon/Lat position
    * @return true if and only if the first n decimal places of p1's and p2's lon and lat values are equal.
    * 
    */
    private boolean almostEqual(PositionGeo p1, PositionGeo p2)
    {
        return false;
        /*
        // OSRM uses 6 decimal places
        DecimalFormat df = new DecimalFormat("###.######");
        
        String p1x = df.format(p1.x());
        String p1y = df.format(p1.y());
        String p2x = df.format(p2.x());
        String p2y = df.format(p2.y());
        
        boolean equalX = p1x.equals(p2x);
        boolean equalY = p1y.equals(p2y);
        boolean almostEqual = equalX && equalY;
        
//		if (DEBUG) System.out.println("almostEqual():\n" + p1.toString() + "\n" + p2.toString() + "\nresult: " + Boolean.toString(almostEqual));
        System.out.flush();
        
        return almostEqual;
        */
    }
    
    private String toString(Point2D.Double p)
    {
        return p.x + " " + p.y;
    }
    
//	private String toString(Vector<Point2D.Double> tp)
//	{
//		String result = "";
//		
//		for (int i = 0; i < tp.size(); i++) {
//			result += this.toString(tp.get(i)) + "\n";
//		}
//		
//		return result;
//	}
    
    private String toString(Vector<PositionGeo> tp)
    {
        String result = "";
        
        for (int i = 0; i < tp.size(); i++) {
            result += tp.get(i).toString() + "\n";
        }
        
        return result;
    }
    
    private void processResponseStatus(String status) throws RSIRequestFailedException
    {
        if (!status.equals("Ok"))
            throw new RSIRequestFailedException("OSRM Request unsuccessful (status " + status + ")");		
    }
}
