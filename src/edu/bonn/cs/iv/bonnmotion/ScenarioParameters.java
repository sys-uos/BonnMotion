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

package edu.bonn.cs.iv.bonnmotion;

import edu.bonn.cs.iv.bonnmotion.printer.Dimension;

public class ScenarioParameters {
	public MobileNode[] nodes;
	public double x = 200.0; /** meters */
	public double y = 200.0; /** meters */
	public double z = 0.0; /** meters */
	public double duration = 600.0; /** seconds */
	public double ignore = 3600.0; /** seconds */
	public long randomSeed = System.currentTimeMillis();
	public String modelName = null;
	public boolean circular = false;
	public double[] aFieldParams = null;
	public AttractorField aField = null;
	public Dimension outputDim = Dimension.TWOD;
	public Dimension calculationDim = Dimension.TWOD;
}