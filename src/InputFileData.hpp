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

#ifndef VME_NMPC_SRC_INPUTFILEDATA_HPP_
#define VME_NMPC_SRC_INPUTFILEDATA_HPP_

#include <string>
#include "typedefs.h"
#include "linear.hpp"

struct InputFileData {
  unsigned nmpcHorizon;
  fptype timeInterval;
  fptype cruiseSpeed;
  fptype Q;
  fptype Q0;
  fptype R;
  fptype sdStepFactor;
  fptype sdConvergenceTolerance;
  unsigned maxSdSteps;
  fptype targetDistanceTolerance;
  std::string jsonLogPath;
  fp_point2d target;

  void load(const std::string &);
  // void save(const std::string &);
};

#endif  // VME_NMPC_SRC_INPUTFILEDATA_HPP_