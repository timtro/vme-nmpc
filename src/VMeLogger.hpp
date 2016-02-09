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

#ifndef VME_NMPC_SRC_DATALOGGER_HPP_
#define VME_NMPC_SRC_DATALOGGER_HPP_

#include "linear.hpp"
#include <memory>

struct VMeCommand;
using up_VMeCommand = std::unique_ptr<VMeCommand>;

/**
 * A template class to provide an interface for loggers. The kernel will
 * organize calls to members promised by this interface. A loger implementing
 * the interface will be a drop-in replacement so that users may add new ways
 * of exporting data from the calculation.
 */
template <typename seedType, typename cmdType>
class NmpcModel;
struct AggregatorInitializer;

class VMeLogger {
 public:
  virtual ~VMeLogger() = default;
  virtual void log_model_state() const noexcept {};
  virtual void log_minimizer_state() const noexcept {};
  virtual void log_constants(const AggregatorInitializer&) const noexcept {};
};

#endif  // VME_NMPC_SRC_DATALOGGER_HPP_
