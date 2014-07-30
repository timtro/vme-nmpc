/*
 * class_SDmin.h
 * Author : Timothy A.V. Teatro
 * Date   : Apr 2014
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2014 - Timothy A.V. Teatro
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

#ifndef __class_SDmin_h__
#define __class_SDmin_h__

#include "struct_nmpc.h"
#include "struct_qnu.h"
#include "struct_Lagr.h"

class optimizer
{

public:
	float* grad;
	float* last_grad;

	optimizer( const qnu*, const Lagr*, const nmpc&, int );
	~optimizer();
	bool iterate( qnu*, Lagr*, nmpc& );
	void teeup();


private:
	float dg;
	int max_iter;
	float tol;
	unsigned int sd_loop;

	void swap_fptr( float** A, float** B );

};

#endif // __class_SDmin_h__