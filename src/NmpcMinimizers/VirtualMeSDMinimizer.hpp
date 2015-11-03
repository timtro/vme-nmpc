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

#ifndef VME_NMPC_VIRTUALMESDMINIMIZER_HPP_
#define VME_NMPC_VIRTUALMESDMINIMIZER_HPP_

#include "../NmpcMinimizer.hpp"
#include "../NmpcModels/VirtualMeModel.hpp"

class VirtualMeSDMinimizer : public NmpcMinimizer {
  VirtualMeModel& model;
  unsigned sdLoopCount{0};
  unsigned maxSteps{1000};
  fptype gradDotPrevGrad{0};
  fptype sdStepFactor{.1};
  fptype convergenceTolerance{.1};
  decltype(model.grad) prevGrad;
  MinimizerCode status{MinimizerCode::idle};

  bool iterate() noexcept;

 public:
  VirtualMeSDMinimizer(VirtualMeModel* model_) : model{*model_} {
    prevGrad = model.grad;
  }

  virtual MinimizerCode solveOptimalControlHorizon() noexcept;
};

#endif  // VME_NMPC_VIRTUALMESDMINIMIZER_HPP_
