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

#include "VMeModel.hpp"
#include "../trig.hpp"
#include "../AggregatorInitializer.hpp"
#include "../VMeCommand.hpp"

VMeModel::VMeModel(AggregatorInitializer& init)
    : N{init.get_nmpcHorizon()},
      T{init.get_timeInterval()},
      cruiseSpeed{init.get_cruiseSpeed()},
      Q{init.get_Q()},
      Q0{init.get_Q0()},
      R{init.get_R()} {
  if (N <= 2) throw HorizonSizeShouldBeSensiblyLarge();

  init.modelBindingSafetyCheck();

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

  //Must be last or destruction of init's unique_ptr causes double destruction.
  init.bindIntoAggregator(this);
}

unsigned VMeModel::getHorizonSize() const noexcept { return N; }

fptype VMeModel::getTargetDistance() const noexcept { return distanceToTarget; }

void VMeModel::setTarget(fp_point2d target){
  absoluteTarget = target;
}

void VMeModel::computeTargetMetrics() {
  targetUnitVector.x = absoluteTarget.x - x[0];
  targetUnitVector.y = absoluteTarget.y - y[0];
  distanceToTarget = std::sqrt(dot(targetUnitVector, targetUnitVector));
  targetUnitVector /= distanceToTarget;
}

void VMeModel::seed(xyth pose) {
  x[0] = pose.x;
  y[0] = pose.y;
  th[0] = pose.th;
  Dx[0] = v[0] * std::cos(th[0]);
  Dy[0] = v[0] * std::sin(th[0]);
  computeTargetMetrics();
}

void VMeModel::seed(xyth pose, fp_point2d target) {
  seed(pose);
  setTarget(target);
  computeTargetMetrics();
}

void VMeModel::computeForecast() noexcept {
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
void VMeModel::computeTrackingErrors() noexcept {
  for (unsigned k = 1; k < N; ++k) {
    ex[k] = x[k] - (x[0] + v[k] * targetUnitVector.x * k * T);
    ey[k] = y[k] - (y[0] + v[k] * targetUnitVector.y * k * T);
  }
}

void VMeModel::computePathPotentialGradient(
    ObstacleContainer& obstacles) noexcept {
  for (unsigned k = 0; k < N; ++k) {
    fp_point2d gradVec = obstacles.gradPhi(fp_point2d{x[k], y[k]});
    DPhiX[k] = gradVec.x;
    DPhiY[k] = gradVec.y;
  }
}

/*!
 * Calculate gradient from ∂J = ∑∂H/∂u ∂u. In doing so, the Lagrange multipliers
 * are computed.
 */
void VMeModel::computeGradient() noexcept {
  gradNorm = 0.;
  px[N - 1] = Q0 * ex[N - 1];
  py[N - 1] = Q0 * ey[N - 1];
  // Dth[N - 2] -= C.dg * R * Dth[N - 2];

  /*!
   * Get the gradient ∂H/∂u_k, for each step, k in the horizon, loop
   * through each k in N. This involves computing the obstacle potential
   * and Lagrange multipliers. Then, the control plan is updated by
   * stepping against the direction of the gradient.
   */
  for (int k = N - 2; k >= 0; --k) {
    px[k] = Q * ex[k] - DPhiX[k] + px[k + 1];
    pDx[k] = px[k + 1] * T;
    py[k] = Q * ey[k] - DPhiY[k] + py[k + 1];
    pDy[k] = py[k + 1] * T;
    pth[k] = pth[k + 1] + pDy[k + 1] * v[k] * std::cos(th[k]) -
             pDx[k + 1] * v[k] * std::sin(th[k]);
    grad[k] = R * Dth[k] + pth[k + 1] * T;
    gradNorm += grad[k] * grad[k];
  }
  gradNorm = sqrt(gradNorm);
}

up_VMeCommand VMeModel::retrieveCommand(int n) const {
  return up_VMeCommand{new VMeV{0, v[n], Dth[n]}};
}

fp_array const& VMeModel::get_x() const noexcept { return x; }

fp_array const& VMeModel::get_Dx() const noexcept { return Dx; }

fp_array const& VMeModel::get_ex() const noexcept { return ex; }

fp_array const& VMeModel::get_y() const noexcept { return y; }

fp_array const& VMeModel::get_Dy() const noexcept { return Dy; }

fp_array const& VMeModel::get_ey() const noexcept { return ey; }

fp_array const& VMeModel::get_v() const noexcept { return v; }

fp_array const& VMeModel::get_th() const noexcept { return th; }

fp_array const& VMeModel::get_Dth() const noexcept { return Dth; }

fp_array const& VMeModel::get_grad() const noexcept { return grad; }

void VMeModel::set_v(fptype velocity) {
  v = velocity;
  Dx[0] = v[0] * std::cos(th[0]);
  Dy[0] = v[0] * std::sin(th[0]);
}