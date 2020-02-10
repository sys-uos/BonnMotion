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

package edu.bonn.cs.iv.util;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Iterator;
import java.lang.Integer;


public class IntegerHashMap extends HashMap<Integer,Object> {
	private static final long serialVersionUID = -9179175672760329379L;

	public Object get(Integer key) {
		Iterator<?> it = this.entrySet().iterator();
		while(it.hasNext()){
			Map.Entry<?,?> entry = (Map.Entry<?,?>)it.next();
			Integer k = ((Integer)entry.getKey());
			if(key.compareTo(k) == 0){
				return entry.getValue();
			}
		}
		return super.get(key);
	}
	
	public void print() {
		Iterator<?> it = this.entrySet().iterator();
		while(it.hasNext()) {
			Map.Entry<?,?> entry = (Map.Entry<?,?>)it.next();
			Integer key = ((Integer)entry.getKey());
			System.out.println("key " + key);
			LinkedList<?> values;
			values = ((LinkedList<?>)entry.getValue());
			for(int i = 0; i < values.size(); i++){
				System.out.println("value " + ((Double)values.get(i)));
			}
		}
	}
}
