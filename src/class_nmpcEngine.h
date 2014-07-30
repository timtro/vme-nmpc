/*
 * class_nmpcEngine.h
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

#ifndef __class_nmpcEngine_h__
#define __class_nmpcEngine_h__

#include "struct_nmpc.h"
#include "struct_qnu.h"
#include "struct_Lagr.h"

class nmpcEngine
{

public:
	//nmpcEngine()
	//~nmpcEngine()

	float tgtdist;

	void init_qu_and_p( qnu* qu, Lagr* p, nmpc& C );
	float stage_cost_execp( qnu* qu, const Lagr* p, const nmpc& C );
	void predict_horizon( qnu* qu, Lagr* p, const nmpc& C );
	void set_tracking_errors ( qnu* qu, Lagr* p, const nmpc& C );
	void get_gradient( qnu* qu, Lagr* p, nmpc& C, float* grad );

private:
	float* dist_from_wall( qnu& qu, wall& W );

};

#endif // __class_nmpcEngine_h__