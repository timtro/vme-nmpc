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

#ifndef VME_NMPC_TESTS_FAKEVIRTUALMEMODEL_HPP_
#define VME_NMPC_TESTS_FAKEVIRTUALMEMODEL_HPP_

#include "../src/NmpcModel.hpp"
#include "../src/VMeCommand.hpp"

class FakeVMeModel
    : public NmpcModel<xyth, fp_point2d, up_VMeCommand> {
  mutable std::string eventHistory{};
  fptype distanceToTarget;
  void recordEvent(char) const;

 public:
  unsigned N = 0;

  FakeVMeModel(unsigned);
  virtual ~FakeVMeModel() = default;
  virtual unsigned getHorizonSize() const;
  virtual fptype getTargetDistance() const noexcept;
  virtual void seed(xyth);
  virtual void seed(xyth, fp_point2d);
  virtual void computeForecast() noexcept;
  virtual void computeTrackingErrors() noexcept;
  virtual void computePathPotentialGradient(ObstacleContainer&) noexcept;
  virtual void computeGradient() noexcept;
  virtual up_VMeCommand getCommand(int) const;
  std::string getEventHistory() const;
};

#endif  // VME_NMPC_TESTS_FAKEVIRTUALMEMODEL_HPP_