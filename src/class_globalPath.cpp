/*
 * class_global_path.cpp
 * Author : Timothy A.V. Teatro
 * Date   : Jan 2014
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

#include <cmath>
#include "class_globalPath.h"
#include <cstdio>
#include <cstdlib>

void globalPath::refreshPath1r( const qnu* qu, const Lagr* p, const nmpc& C )
{
	point tp; // scrap point for working.

	Path.clear();
	tp.x = qu[0].x;
	tp.y = qu[0].y;
	Path.push_back( tp );
	tp.x = C.walls[C.nwalls - 1].x1;
	tp.y = C.walls[C.nwalls - 1].y1;
	if ( qu[0].y <= tp.y )
		Path.push_back( tp );

	tp.x = C.tgt[0];
	tp.y = C.tgt[1];
	Path.push_back( tp );
}

void globalPath::refreshPath2r( const qnu* qu, const Lagr* p, const nmpc& C )
{
	point tp[4]; // scrap point for working.

	Path.clear();
	tp[0].x = qu[0].x;
	tp[0].y = qu[0].y;
	tp[1].x = C.walls[C.nwalls - 2].x1;
	tp[1].y = C.walls[C.nwalls - 2].y1;
	tp[2].x = C.walls[C.nwalls - 1].x0;
	tp[2].y = C.walls[C.nwalls - 1].y0;
	Path.push_back( tp[0] );
	if ( qu[C.N - 1].y <= tp[1].y )
		if( atan2( tp[1].y - tp[0].y, tp[1].x - tp[0].x ) < atan2( tp[2].y - tp[0].y, tp[2].x - tp[0].x ) )
			Path.push_back( tp[1] );
	if ( qu[C.N - 1].y <= tp[2].y )
		if ( atan2( tp[2].y - tp[0].y, tp[2].x - tp[0].x ) > atan2( gtgt.y - tp[0].y, gtgt.x - tp[0].x ) )
			Path.push_back( tp[2] );
	Path.push_back( gtgt );
}

void globalPath::refreshPath2rEP( const qnu* qu, const Lagr* p, const nmpc& C )
{
	point tp[4]; // scrap point for working.

	Path.clear();
	tp[0].x = qu[C.N - 1].x;
	tp[0].y = qu[C.N - 1].y;
	tp[1].x = C.walls[C.nwalls - 2].x1;
	tp[1].y = C.walls[C.nwalls - 2].y1;
	tp[2].x = C.walls[C.nwalls - 1].x0;
	tp[2].y = C.walls[C.nwalls - 1].y0;
	Path.push_back( tp[0] );
	if ( qu[C.N - 1].y <= tp[1].y )
		if( atan2( tp[1].y - tp[0].y, tp[1].x - tp[0].x ) < atan2( tp[2].y - tp[0].y, tp[2].x - tp[0].x ) )
			Path.push_back( tp[1] );
	if ( qu[C.N - 1].y <= tp[2].y )
		if ( atan2( tp[2].y - tp[0].y, tp[2].x - tp[0].x ) > atan2( gtgt.y - tp[0].y, gtgt.x - tp[0].x ) )
			Path.push_back( tp[2] );
	Path.push_back( gtgt );
}

void globalPath::samplePath( const nmpc& C )
{

	float dirx, diry, dist, dx;
	point tp;

	sPath.clear();

	for ( unsigned int k = 0; k < Path.size() - 1; ++k )
	{
		dirx = Path[k + 1].x - Path[k].x;
		diry = Path[k + 1].y - Path[k].y;
		// TODO: Store dist this to use in loop terminator.
		dist = hypot( dirx, diry );
		dirx /= dist;
		diry /= dist;

		dx = C.cruising_speed * C.T;
		tp.x = Path[k].x;
		tp.y = Path[k].y;
		sPath.push_back( tp );
		do
		{
			tp.x += dx * dirx;
			tp.y += dx * diry;
			sPath.push_back( tp );
			dist = hypot( tp.x - Path[k + 1].x, tp.y - Path[k + 1].y );
		}
		while ( dist > dx );
	}
}

void globalPath::setExEy( qnu* qu, Lagr* p, const nmpc& C )
{
	// Identify which part of the sPath we are closest to to start.

	float dist;
	float mindist = hypot( sPath[0].x - qu[0].x, sPath[0].y - qu[0].y );
	int mink = 0; // The index of the point in sPath closest to the robot.
	int overflow; // If the robot is close to the end of the global path...

	for ( unsigned int k = 1; k < sPath.size(); ++k )
	{
		dist = hypot( qu[0].x - sPath[k].x, qu[0].y - sPath[k].y );
		if ( dist < mindist )
		{
			mindist = dist;
			mink = k;
		}
	}

	//
	// If the horizon is longer than the distance remaining in then
	// path, then we need to handle it in a special way. Test for this.
	//
	if ( ( overflow = mink + C.N - sPath.size() ) > 0 )
	{
		for ( unsigned int j = 1; j < C.N - overflow; ++j )
		{
			p[j].ex = qu[j].x - sPath[mink + j].x;
			p[j].ey = qu[j].y - sPath[mink + j].y;
		}

		float dirx =  sPath.back().x - qu[0].x;
		float diry =  sPath.back().y - qu[0].y;
		float dx = C.cruising_speed * C.T;
		dist = hypot( dirx, diry );
		dirx /= dist;
		diry /= dist;

		for ( unsigned int j = C.N - overflow; j < C.N; ++j )
		{
			// Error of the overflowed horizon points should direct the
			// path past the target. I tried setting this to zero, but
			// that removes any influence from these points on the cost.

			p[j].ex = qu[j].x - ( qu[0].x + dx * j * dirx );
			p[j].ey = qu[j].y - ( qu[0].y + dx * j * diry );
		}
	}
	else
	{
		for ( unsigned int j = 1; j < C.N; ++j )
		{
			p[j].ex = qu[j].x - sPath[mink + j].x;
			p[j].ey = qu[j].y - sPath[mink + j].y;
		}
	}
}

void globalPath::getEndPenalty( const qnu* qu, const nmpc& C, point* tp )
{
	float dist;
	float mindist = hypot( sPath[0].x - qu[C.N - 1].x, sPath[0].y - qu[C.N - 1].y );
	int mink = 0;

	for ( unsigned int k = 1; k < sPath.size(); ++k )
	{
		dist = hypot( qu[C.N - 1].x - sPath[k].x, qu[C.N - 1].y - sPath[k].y );
		if ( dist < mindist )
		{
			mindist = dist;
			mink = k;
		}
	}
	tp->x = qu[C.N - 1].x - sPath[mink].x;
	tp->y = qu[C.N - 1].y - sPath[mink].y;
}

float globalPath::length()
{
	float len = 0.;
	for ( unsigned int k = 0; k < Path.size() - 1; ++k )
	{
		len += hypot( Path[k + 1].x - Path[k].x, Path[k + 1].y - Path[k].y );
	}
	return len;
}

void globalPath::GPE( const qnu* qu, const nmpc& C, point* tp )
{
	float len, dist;
	len = 0.;

	for ( unsigned int k = 0; k < Path.size() - 1; ++k )
	{
		len += hypot( Path[k + 1].x - Path[k].x, Path[k + 1].y - Path[k].y );
	}
	dist = hypot( Path[1].x - qu[C.N - 1].x, Path[1].y - qu[C.N - 1].y );
	if ( hypot( qu[0].x - Path[Path.size() - 1].x, qu[0].y - Path[Path.size() - 1].y ) < C.cruising_speed * C.N * C.T  )
	{
		tp->x = 0.;
		tp->y = 0.;
	}
	else
	{
		tp->x = -len * ( Path[1].x - qu[C.N - 1].x ) / dist;
		tp->y = -len * ( Path[1].y - qu[C.N - 1].y ) / dist;
	}
}

void globalPath::AE( const qnu* qu, const nmpc& C, point* tp )
{
	int end = Path.size() - 1;
	tp->x = qu[C.N - 1].x - Path[end].x;
	tp->y = qu[C.N - 1].y - Path[end].y;
}

int globalPath::GetG( const nmpc& C )
{
	float len, dist;
	len = 0.;

	for ( unsigned int k = 0; k < Path.size() - 1; ++k )
	{
		len += hypot( Path[k + 1].x - Path[k].x, Path[k + 1].y - Path[k].y );
	}

	return int( len / ( C.cruising_speed * C.T ) );
}