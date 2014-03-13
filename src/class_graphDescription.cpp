/*
 *
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

#include <cstdio>
#include <cmath>
#include <vector>
#include "class_graphDescription.h"
#include "class_graphNode.h"

graphDescription::graphDescription ( int cr, std::vector<float> euclid_ranges, float ds )
{
	circleRadius = cr;
	Xmin = euclid_ranges[0];
	Ymin = euclid_ranges[1];
	Xmax = euclid_ranges[2];
	Ymax = euclid_ranges[3];
	Ximin = ( int ) ( Xmin / ds );
	Yimin = ( int ) ( Ymin / ds );
	Ximax = ( int ) ( Xmax / ds );
	Yimax = ( int ) ( Ymax / ds );

}

graphDescription::graphDescription ()
{
	circleRadius = 10;
	Xmin = -20;
	Xmax = 20;
	Ymin = -20;
	Ymax = 20;
	ds = .25;
	Ximin = ( int ) ( Xmin / ds );
	Yimin = ( int ) ( Ymin / ds );
	Ximax = ( int ) ( Xmax / ds );
	Yimax = ( int ) ( Ymax / ds );
}

int graphDescription::getHashBin( graphNode& n )
{
	// Use the absolute value of x-coordinate
	// as hash bin counter. Not a good choice though!
	return ( ( int ) std::fabs( n.x ) );
}

bool graphDescription::isAccessible( graphNode& n )
{
	// printf( "[%d, %d] in bound [%d, %d]--[%d, %d]\n", n.x, n.y, Ximin, Yimin, Yimax, Ximax );
	static int objx = 13;
	static int objy = 20;
	//printf( "%d, %d\n", objx, objy );
	int sx = objx - n.x;
	int sy = objy - n.y;

	return ( ( ( sx * sx + sy * sy ) > circleRadius )
	         && ( n.x >= Ximin && n.x <= Ximax && n.y >= Yimin && n.y <= Yimax ) );
}

void graphDescription::getSuccessors( graphNode& n, std::vector<graphNode>* s, std::vector<double>* c ) // Define a 8-connected graph
{
	// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
	graphNode tn;
	s->clear();
	c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
	for ( int a = -1; a <= 1; a++ )
		for ( int b = -1; b <= 1; b++ )
		{
			if ( a == 0 && b == 0 ) continue;
			tn.x = n.x + a;
			tn.y = n.y + b;
			s->push_back( tn );
			c->push_back( sqrt( ( double )( a * a + b * b ) ) );
		}
}

double graphDescription::getHeuristics( graphNode& n1, graphNode& n2 )
{
	int dx = std::abs( n1.x - n2.x );
	int dy = std::abs( n1.y - n2.y );
	return ( sqrt( ( double )( dx * dx + dy * dy ) ) ); // Euclidean distance as heuristics
}