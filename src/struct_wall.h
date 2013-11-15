/*
 * struct_wall.h
 * Author : Timothy A.V. Teatro
 * Date   : 2013-11-14
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

#ifndef __struct_wall_h__
#define __struct_wall_h__

typedef struct wall_tag
{
	float x0;
	float y0;
	float x1;
	float y1;
} wall;

#endif // __struct_wall_h__