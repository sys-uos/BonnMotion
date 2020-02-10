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

package edu.bonn.cs.iv.bonnmotion.models.smooth;

public class ThreeDDoubleArrayWrapper {
	private TwoDDoubleWrapper[] data;
	private int x;
	private int y;
	private int z;

	public ThreeDDoubleArrayWrapper(int length) {
		data = new TwoDDoubleWrapper[length];
		for (int i = 0; i < length; i++)
			data[i] = new TwoDDoubleWrapper(length);
		this.x = length;
		this.y = length;
		this.z = length;
	}

	public ThreeDDoubleArrayWrapper(int x, int y, int z) {
		data = new TwoDDoubleWrapper[x];
		for (int i = 0; i < x; i++)
			data[i] = new TwoDDoubleWrapper(y, z);
		this.x = x;
		this.y = y;
		this.z = z;
	}

	public double get(int x, int y, int z) {
		return data[x].get(y, z);
	}

	public void set(int x, int y, int z, double newDouble) throws WrapperMaximumLengthExceededException {
		if(x>=data.length){
			throw new WrapperMaximumLengthExceededException("Top-level wrapper x index too large");
		}
		data[x].set(y, z, newDouble);
	}

	public boolean isSquare() {
		return (x == y) && (y == z);
	}
}
