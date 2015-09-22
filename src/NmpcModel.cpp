/*
 * vme-nmpc/src/NmpcModel.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-09-15
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 - Timothy A.V. Teatro
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

#include "NmpcModel.hpp"
#include "trig.hpp"


/*
Make state space styff accessible through agetters. Lagrange portion doesent
need utesting and can be fkushed out through gradient.
 */

NmpcModel::NmpcModel(NmpcInitPkg& ini) :
  N{ini.N}, m{ini.m}, n{ini.n}, T{ini.T}, dg{ini.dg},
  cruising_speed{ini.cruising_speed}, Q{ini.Q}, Q0{ini.Q0}, R{ini.R} {
  size_t size = static_cast<size_t>(N);

  // State Vector:
  x = std::valarray<float>(0.f, size);
  Dx = std::valarray<float>(0.f, size);
  y = std::valarray<float>(0.f, size);
  Dy = std::valarray<float>(0.f, size);
  th = std::valarray<float>(0.f, size);
  // Control vector:
  Dth = std::valarray<float>(0.f, size);
  // Other:
  v = std::valarray<float>(cruising_speed, size);
  ex = std::valarray<float>(0.f, size);
  ey = std::valarray<float>(0.f, size);
  // Lagrange Multipliers:
  px = std::valarray<float>(0.f, size);
  pDx = std::valarray<float>(0.f, size);
  py = std::valarray<float>(0.f, size);
  pDy = std::valarray<float>(0.f, size);
  pth = std::valarray<float>(0.f, size);
  // Gradients:
  grad = std::valarray<float>(0.f, size);
  last_grad = std::valarray<float>(0.f, size);
}

void NmpcModel::seed() {
  Dx[0] = v[0] * std::cos(th[0]);
  Dy[0] = v[0] * std::sin(th[0]);
}

void NmpcModel::seed(XYVTh<float> seed) {
  x[0] = seed.x;
  y[0] = seed.y;
  v[0] = seed.v;
  th[0] = degToRad(seed.th);
  this->seed();
}

void NmpcModel::forecast() {
  for (int k = 1; k < N; ++k) {
    th[k] = th[k - 1] + Dth[k - 1] * T;
    x[k] = x[k - 1] + Dx[k - 1] * T;
    Dx[k] = v[k] * std::cos(th[k]);
    y[k] = y[k - 1] + Dy[k - 1] * T;
    Dy[k] = v[k] * std::sin(th[k]);
  }
}

// /*!
//  * This function sets the tracking error. This function is overloaded,
//  * and in other flavours, will take global paths.
//  *
//  * This is vanilla, and just tracks the straight line at the cruising speed.
//  */
// void NmpcEngine::set_tracking_errors(qnu* qu, Lagr* p, const nmpc& C) {
//   float dirx = targets.top().x - x[0];
//   float diry = targets.top().y - y[0];
//   // TODO: Store dist this to use in loop terminator.
//   float dist = sqrt(dirx * dirx + diry * diry);
//   dirx /= dist;
//   diry /= dist;

//   for(unsigned int k = 1; k < N; ++k) {
//     ex[k] = x[k] - (x[0] + cruising_speed * dirx * k * T);
//     ey[k] = y[k] - (y[0] + cruising_speed * diry * k * T);
//   }
// }

// /*!
//  * Calculate gradient from ∂J = ∑∂H/∂u ∂u. In doing so, the Lagrange multipliers
//  * are computed. The norm of the gradient vector is also sotred in the N+1th
//  * element of the gradient array.
//  */
// void NmpcEngine::get_gradient() {

//   int k;
//   unsigned int j;
//   float denom, difx, dify, gDth;

//   grad[N] = 0.;

//   px[N - 1] = Q0 * ex[N - 1];
//   py[N - 1] = Q0 * ey[N - 1];
//   Dth[N - 2] -= dg * R * Dth[N - 2];


//   //TODO(TT): Try auto here:
//   // std::tie(std::valarray<float> DPhi_x, std::valarray<float> DPhi_y) =
//   // obstacles.grad_phi(x, y);
//   std::valarray<float> gphi_x;
//   std::valarray<float> gphi_y;
//   std::tie(gphi_x, gphi_y) = obstacles.grad_phi(x, y);

//   /*!
//    * Get the gradient ∂H/∂u_k, for each step, k in the horizon, loop
//    * through each k in N. This involves computing the obstacle potential
//    * and Lagrange multipliers. Then, the control plan is updated by
//    * stepping against the direction of the gradient.
//    */
//   for(k = N - 2; k >= 0; --k) {

//     // Compute the Lagrange multipliers:
//     px[k] = Q * ex[k] - gphi_x[k] + px[k + 1];
//     pDx[k] = px[k + 1] * T;
//     py[k] = Q * ey[k] - gphi_y[k] + py[k + 1];
//     pDy[k] = py[k + 1] * T;
//     pth[k] = pth[k + 1] + pDy[k + 1] * v[k] * std::cos(th[k])
//              - pDx[k + 1] * v[k] * std::sin(th[k]);
//     grad[k] = R * Dth[k] + pth[k + 1] * T;
//     grad[N] += grad[k] * grad[k];
//   }
//   grad[N] = sqrt(grad[N]);
// }