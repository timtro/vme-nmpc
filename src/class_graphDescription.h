/*
 * class_graphDescription.h
 * Author : Timothy A.V. Teatro
 * Date   : Mar 2014
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2013, 2014 - Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __class_graphDescription_h__
#define __class_graphDescription_h__

#include <vector>
#include "class_graphNode.h"

class graphDescription
{
public:

	graphDescription ( int cr, std::vector<float> euclid_ranges, float ds );
	graphDescription ();

	int getHashBin( graphNode& n );
	bool isAccessible( graphNode& n );
	void getSuccessors( graphNode& n, std::vector<graphNode>* s, std::vector<double>* c );
	double getHeuristics( graphNode& n1, graphNode& n2 );

private:

	int circleRadius;
	int Ximin, Ximax, Yimin, Yimax;
	float Xmin, Xmax, Ymin, Ymax;
	float ds;

};

#endif // __class_graphDescription_h__