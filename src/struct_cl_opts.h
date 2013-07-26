/*
 * struct_cl_opts.h
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-28
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

#ifndef STRUCT_CL_OPTS_H_
#define STRUCT_CL_OPTS_H_

typedef struct cl_opts_tag
{
	bool selec_verbose;
	bool selec_quiet;
	bool selec_state_and_error_SE;
	bool selec_lagrange_grad_LG;
	bool selec_SD_converged_SD;
	bool selec_target_reached_TR;
	bool selec_sim;
} cl_opts;

#endif /* STRUCT_CL_OPTS_H_ */
