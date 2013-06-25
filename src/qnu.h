/*
 * qnu.h
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-04
 *
 * This file is part of vme-nmpc
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

#ifndef __QNU_H__
#define __QNU_H__

typedef struct qnutag {
  float x;
  float Dx;
  float y;
  float Dy;
  float th;
  float Dth;
  float v;
} qnu;

typedef struct Lagrtag {
  float ex;
  float ey;
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  float sintk;
  float costk;
} Lagr;

#endif /* __QNU_H_ */
