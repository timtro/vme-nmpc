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

#ifndef VME_NMPC_SRC_NMPCMODELS_VIRTUALMEMODEL_HPP_
#define VME_NMPC_SRC_NMPCMODELS_VIRTUALMEMODEL_HPP_

#include "../NmpcModel.hpp"
#include "../Obstacle.hpp"
#include "../VirtualMeCommand.hpp"
#include "../NmpcInitPkg.hpp"

class VirtualMeModel
    : public NmpcModel<xyvth, fp_point2d, up_VirtualMeCommand> {
  fptype distanceToTarget;
  fp_point2d targetVector;

 public:
  unsigned N;
  unsigned m;
  unsigned n;
  fptype T;
  fptype cruiseSpeed;
  fptype Q;
  fptype Q0;
  fptype R;
  //! The x-coordinate.
  fp_array x;
  //! The time rate-of-change of x.
  fp_array Dx;
  //! The y-coordinate.
  fp_array y;
  //! The time rate-of-change of y.
  fp_array Dy;
  //! The angle from the x-axis of the direction of travel.
  fp_array th;
  //! The steering rate. That is, the time rate-of-change of th.
  fp_array Dth;
  //! The radial component of speed.
  fp_array v;
  //! The error of the x-coordinate.
  fp_array ex;
  //! The error of the y-coordinate.
  fp_array ey;
  // Potential gradient at each point in path.
  fp_array DPhiX;
  fp_array DPhiY;

  // The Lagrange multipliers
  fp_array px;
  fp_array pDx;
  fp_array py;
  fp_array pDy;
  fp_array pth;

  fptype gradNorm = 0.f;
  fp_array grad;

  VirtualMeModel(NmpcInitPkg &);
  virtual ~VirtualMeModel() = default;

  virtual unsigned getHorizonSize() const;
  virtual fptype getTargetDistance();
  virtual void seed(xyvth, fp_point2d);
  virtual void seed(xyvth);
  virtual void computeForecast() noexcept;
  virtual void computeTrackingErrors() noexcept;
  virtual void computePathPotentialGradient(
      ObstacleContainer &obstacles) noexcept;
  void computeLagrageMultipliers() noexcept;
  virtual void computeGradient() noexcept;
  virtual up_VirtualMeCommand getCommand(int);
};

#endif  // VME_NMPC_SRC_NMPCMODELS_VIRTUALMEMODEL_HPP_