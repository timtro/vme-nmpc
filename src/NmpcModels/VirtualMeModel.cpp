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

// TODO(TT): Test speed of storing sintk vs letting the compiler optimize it.

// NB(TT to TT): to convert from old notation:
// s/qu\[([^\]]*)\]\.([a-zA-Z0-9]*)/\2\[\1\]/g

#include "VirtualMeModel.hpp"
#include "../trig.hpp"
#include "../NmpcInitPkg.hpp"

VirtualMeModel::VirtualMeModel(NmpcInitPkg& ini)
    : N{ini.N},
      m{ini.m},
      n{ini.n},
      T{ini.T},
      cruiseSpeed{ini.cruiseSpeed},
      Q{ini.Q},
      Q0{ini.Q0},
      R{ini.R} {
  size_t horizonSize = static_cast<size_t>(N);

  // State Vector:
  x = fpArray(0.f, horizonSize);
  Dx = fpArray(0.f, horizonSize);
  y = fpArray(0.f, horizonSize);
  Dy = fpArray(0.f, horizonSize);
  th = fpArray(0.f, horizonSize);
  // Control vector:
  Dth = fpArray(0.f, horizonSize - 1);
  // Other:
  v = fpArray(cruiseSpeed, horizonSize);
  ex = fpArray(0.f, horizonSize);
  ey = fpArray(0.f, horizonSize);
  DPhiX = fpArray(0.f, horizonSize);
  DPhiY = fpArray(0.f, horizonSize);
  // Lagrange Multipliers:
  px = fpArray(0.f, horizonSize);
  pDx = fpArray(0.f, horizonSize);
  py = fpArray(0.f, horizonSize);
  pDy = fpArray(0.f, horizonSize);
  pth = fpArray(0.f, horizonSize);
  // Gradients:
  grad = fpArray(0.f, horizonSize - 1);
  prevGrad = fpArray(0.f, horizonSize - 1);
}

void VirtualMeModel::seed(xyvth pose, Point2R target) {
  x[0] = pose.x;
  y[0] = pose.y;
  th[0] = pose.th;
  Dx[0] = v[0] * std::cos(th[0]);
  Dy[0] = v[0] * std::sin(th[0]);

  targetVector_.x = target.x - x[0];
  targetVector_.y = target.y - y[0];
  distanceToTarget_ = std::sqrt(dot(targetVector_, targetVector_));
  targetVector_ /= distanceToTarget_;
}

void VirtualMeModel::seed(xyvth pose) {
  x[0] = pose.x;
  y[0] = pose.y;
  th[0] = degToRad(pose.th);
  Dx[0] = v[0] * std::cos(th[0]);
  Dy[0] = v[0] * std::sin(th[0]);
}

void VirtualMeModel::forecast() {
  for (unsigned k = 1; k < N; ++k) {
    th[k] = th[k - 1] + Dth[k - 1] * T;
    x[k] = x[k - 1] + Dx[k - 1] * T;
    Dx[k] = v[k] * std::cos(th[k]);
    y[k] = y[k - 1] + Dy[k - 1] * T;
    Dy[k] = v[k] * std::sin(th[k]);
  }
}

/*!
 * This is vanilla, and just tracks the straight line at the cruising speed.
 */
void VirtualMeModel::setTrackingErrors() {
  for (unsigned k = 1; k < N; ++k) {
    ex[k] = x[k] - (x[0] + cruiseSpeed * targetVector_.x * k * T);
    ey[k] = y[k] - (y[0] + cruiseSpeed * targetVector_.y * k * T);
  }
}

void VirtualMeModel::computePathPotentialGradient(ObstacleStack& obstacles) {
  for (unsigned k = 0; k < N; ++k) {
    Point2R gradVec = obstacles.gradPhi(Point2R{x[k], y[k]});
    DPhiX[k] = gradVec.x;
    DPhiY[k] = gradVec.y;
  }
}

void VirtualMeModel::computeLagrageMultipliers() {
  px[N - 1] = Q0 * ex[N - 1];
  py[N - 1] = Q0 * ey[N - 1];

  for (unsigned k = N - 2; k == 0; --k) {
    // Compute the Lagrange multipliers:
    px[k] = Q * ex[k] - DPhiX[k] + px[k + 1];
    pDx[k] = px[k + 1] * T;
    py[k] = Q * ey[k] - DPhiY[k] + py[k + 1];
    pDy[k] = py[k + 1] * T;
    pth[k] = pth[k + 1] + pDy[k + 1] * v[k] * std::cos(th[k]) -
             pDx[k + 1] * v[k] * std::sin(th[k]);
  }
}

/*!
 * Calculate gradient from ∂J = ∑∂H/∂u ∂u. In doing so, the Lagrange multipliers
 * are computed.
 */
void VirtualMeModel::computeGradient() {
  grad[N - 1] = R * Dth[N - 2] + pth[N - 1] * T;  // segfault?
  computeLagrageMultipliers();
  gradNorm = 0.f;
  for (unsigned k = N - 2; k == 0; --k) {
    grad[k] = R * Dth[k] + pth[k + 1] * T;
    gradNorm += grad[k] * grad[k];
  }
  gradNorm = std::sqrt(gradNorm);
}

fptype VirtualMeModel::distanceToTarget() { return distanceToTarget_; }
