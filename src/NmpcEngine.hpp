/* This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 Timothy A.V. Teatro - All rights Reserved
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __VME_NMPC_SRC_NMPCENGINE_HPP__
#define __VME_NMPC_SRC_NMPCENGINE_HPP__

#include "NmpcModel.hpp"
#include "NmpcMinimizer.hpp"

#include "typedefs.h"

class NmpcEngine {
  std::unique_ptr<NmpcModel> model;
  std::unique_ptr<NmpcMinimizer> minimizer;
 public:
  NmpcEngine(NmpcInitPkg &ini, std::unique_ptr<NmpcModel> model,
             std::unique_ptr<NmpcMinimizer> minimizer);
};

#endif // __VME_NMPC_SRC_NMPCENGINE_HPP__