/*
 * struct_intok.h
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



#ifndef STRUCT_INTOK_H_
#define STRUCT_INTOK_H_

/*!
 * A structure that holds the symbol for an input variable, and a bool that is
 * set true when a value is recorded for that symbol from the input file.
 */
typedef struct intok_tag {
  const char* token;
  bool saw_tok;
} intok;


#endif /* STRUCT_INTOK_H_ */
