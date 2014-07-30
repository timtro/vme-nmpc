/*
 * class_global_path.h
 * Author : Timothy A.V. Teatro
 * Date   : Jan 2014
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

#ifndef __class_globalPath_h__
#define __class_globalPath_h__

#include <vector>
#include "struct_nmpc.h"
#include "struct_qnu.h"
#include "struct_Lagr.h"

typedef struct point_tag
{
	float x;
	float y;
} point;

class globalPath
{

public:
	std::vector<point> Path;
	std::vector<point> sPath;
	point gtgt;

	void refreshPath1r( const qnu* qu, const Lagr* p, const nmpc& C );
	void refreshPath2r( const qnu* qu, const Lagr* p, const nmpc& C );
	void refreshPath2rEP( const qnu* qu, const Lagr* p, const nmpc& C );
	void samplePath( const nmpc& C );
	void setExEy( qnu* qu, Lagr* p, const nmpc& C );
	void getEndPenalty( const qnu* qu, const nmpc& C, point* tp );
	float length();
	void GPE( const qnu* qu, const nmpc& C, point* tp );
	void AE( const qnu* qu, const nmpc& C, point* tp );
	int GetG( const nmpc& C );
};

#endif // __class_globalPath_h__