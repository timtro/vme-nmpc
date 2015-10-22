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

#ifndef __VME_NMPC_TESTS_FAKEVIRTUALMEMODEL_HPP__
#define __VME_NMPC_TESTS_FAKEVIRTUALMEMODEL_HPP__

#include "../src/NmpcModel.hpp"
#include "../src/VirtualMeCommand.hpp"

class FakeVirtualMeModel
    : public NmpcModel<xyvth, Point2R, upVirtualMeCommand> {
  std::string eventHistory{};
  fptype distanceToTarget;
  void recordEvent(char);

 public:
  unsigned N = 0;

  FakeVirtualMeModel(unsigned);
  virtual ~FakeVirtualMeModel() = default;
  virtual void seed(xyvth);
  virtual void seed(xyvth, Point2R);
  virtual void forecast();
  virtual void setTrackingErrors();
  virtual void computePathPotentialGradient(ObstacleStack&);
  virtual void computeGradient();
  virtual fptype getTargetDistance();
  virtual upVirtualMeCommand getCommand(int);
  std::string getEventHistory();
  virtual unsigned getHorizonSize() const;
};

#endif  // __VME_NMPC_TESTS_FAKEVIRTUALMEMODEL_HPP__