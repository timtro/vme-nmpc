/*
 * class_graphNode.h
 * Author : Timothy A.V. Teatro
 * Date   : Mar 2014
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2013 - Timothy A.V. Teatro
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

#ifndef __class_graphNode_h__
#define __class_graphNode_h__

class graphNode
{
public:
	int x, y; // Profiling observation: integer coordinates, hence operator==,
	//  makes the search significantly faster (almost 10 folds than double)
	bool operator==( const graphNode& n )
	{
		return ( x == n.x && y == n.y );
	}; // This must be defined for the node
};

#endif // __class_graphNode_h__