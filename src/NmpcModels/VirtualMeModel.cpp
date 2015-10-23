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
#include "../VirtualMeCommand.hpp"

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
  x = fp_array(0.f, horizonSize);
  Dx = fp_array(0.f, horizonSize);
  y = fp_array(0.f, horizonSize);
  Dy = fp_array(0.f, horizonSize);
  th = fp_array(0.f, horizonSize);
  // Control vector:
  Dth = fp_array(0.f, horizonSize - 1);
  // Other:
  v = fp_array(cruiseSpeed, horizonSize);
  ex = fp_array(0.f, horizonSize);
  ey = fp_array(0.f, horizonSize);
  DPhiX = fp_array(0.f, horizonSize);
  DPhiY = fp_array(0.f, horizonSize);
  // Lagrange Multipliers:
  px = fp_array(0.f, horizonSize);
  pDx = fp_array(0.f, horizonSize);
  py = fp_array(0.f, horizonSize);
  pDy = fp_array(0.f, horizonSize);
  pth = fp_array(0.f, horizonSize);
  // Gradients:
  grad = fp_array(0.f, horizonSize - 1);
}

unsigned VirtualMeModel::getHorizonSize() const { return N; }

fptype VirtualMeModel::getTargetDistance() { return distanceToTarget; }

void VirtualMeModel::seed(xyvth pose, fp_point2d target) {
  x[0] = pose.x;
  y[0] = pose.y;
  th[0] = pose.th;
  Dx[0] = v[0] * std::cos(th[0]);
  Dy[0] = v[0] * std::sin(th[0]);

  targetVector.x = target.x - x[0];
  targetVector.y = target.y - y[0];
  distanceToTarget = std::sqrt(dot(targetVector, targetVector));
  targetVector /= distanceToTarget;
}

void VirtualMeModel::seed(xyvth pose) {
  x[0] = pose.x;
  y[0] = pose.y;
  th[0] = degToRad(pose.th);
  Dx[0] = v[0] * std::cos(th[0]);
  Dy[0] = v[0] * std::sin(th[0]);
}

void VirtualMeModel::computeForecast() noexcept {
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
void VirtualMeModel::computeTrackingErrors() noexcept {
  for (unsigned k = 1; k < N; ++k) {
    ex[k] = x[k] - (x[0] + cruiseSpeed * targetVector.x * k * T);
    ey[k] = y[k] - (y[0] + cruiseSpeed * targetVector.y * k * T);
  }
}

void VirtualMeModel::computePathPotentialGradient(
    ObstacleContainer& obstacles) noexcept {
  for (unsigned k = 0; k < N; ++k) {
    fp_point2d gradVec = obstacles.gradPhi(fp_point2d{x[k], y[k]});
    DPhiX[k] = gradVec.x;
    DPhiY[k] = gradVec.y;
  }
}

void VirtualMeModel::computeLagrageMultipliers() noexcept {
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
void VirtualMeModel::computeGradient() noexcept {
  grad[N - 1] = R * Dth[N - 2] + pth[N - 1] * T;  // segfault?
  computeLagrageMultipliers();
  gradNorm = 0.f;
  for (unsigned k = N - 2; k == 0; --k) {
    grad[k] = R * Dth[k] + pth[k + 1] * T;
    gradNorm += grad[k] * grad[k];
  }
  gradNorm = std::sqrt(gradNorm);
}

up_VirtualMeCommand VirtualMeModel::getCommand(int n) {
  return up_VirtualMeCommand{new VMeV{0, v[n], Dth[n]}};
}