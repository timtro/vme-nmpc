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

#ifndef __VME_NMPC_SRC_NMPCMODELS_VIRTUALMEMODEL_HPP__
#define __VME_NMPC_SRC_NMPCMODELS_VIRTUALMEMODEL_HPP__

#include "../NmpcModel.hpp"
#include "../Obstacle.hpp"
#include "../NmpcInitPkg.hpp"

class VirtualMeModel : public NmpcModel {
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
  fpArray x;
  //! The time rate-of-change of x.
  fpArray Dx;
  //! The y-coordinate.
  fpArray y;
  //! The time rate-of-change of y.
  fpArray Dy;
  //! The angle from the x-axis of the direction of travel.
  fpArray th;
  //! The steering rate. That is, the time rate-of-change of th.
  fpArray Dth;
  //! The radial component of speed.
  fpArray v;
  //! The error of the x-coordinate.
  fpArray ex;
  //! The error of the y-coordinate.
  fpArray ey;
  // Potential gradient at each point in path.
  fpArray DPhiX;
  fpArray DPhiY;

  // The Lagrange multipliers
  fpArray px;
  fpArray pDx;
  fpArray py;
  fpArray pDy;
  fpArray pth;

  fptype gradNorm = 0.f;
  fpArray grad;
  fpArray prevGrad;

  VirtualMeModel(NmpcInitPkg &);
  virtual ~VirtualMeModel() = default;
  void computeLagrageMultipliers();

  virtual void seed();
  virtual void seed(XYVTh<fptype>);
  virtual void forecast();
  virtual void setTrackingErrors(Point2R target);
  virtual void computePathPotentialGradient(ObstacleStack &obstacles);
  virtual void computeGradient();
};

#endif  // __VME_NMPC_SRC_NMPCMODELS_VIRTUALMEMODEL_HPP__