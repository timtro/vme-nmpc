/*
 * struct_cmd.h
 * Author : Timothy A.V. Teatro
 * Date   : Jul 23, 2013
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

/*
 * This struct is just meant to hold the commands that will be executed at the
 * end of the SD loop. At the end of the SD loop, q, u and p are shifted by
 * the control horizon which destroys these data. This truct will hold it so
 * that it can be executed after the close of the SD loop.
 */

#ifndef STRUCT_CMD_H_
#define STRUCT_CMD_H_

typedef struct cmd_tag
{
	float v;
	float Dth;
} cmd;

#endif /* STRUCT_CMD_H_ */
