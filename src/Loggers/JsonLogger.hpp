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

#ifndef VME_NMPC_SRC_LOGGERS_VIRTUALMELOGGER_HPP_
#define VME_NMPC_SRC_LOGGERS_VIRTUALMELOGGER_HPP_

#include "../VMeLogger.hpp"
#include "../CFileContainer.hpp"

class VMeModel;
class VMeNaiveSdMinimizer;

class JsonLogger : public VMeLogger {
  VMeModel* model{nullptr};
  FILE* fp_out{stdout};
  std::unique_ptr<CFileContainer> logFile;
  mutable bool printedFirstObject{false};

 public:
  JsonLogger(NmpcModel<xyth, fp_point2d, up_VMeCommand>*);
  JsonLogger(NmpcModel<xyth, fp_point2d, up_VMeCommand>*, FILE*);
  JsonLogger(NmpcModel<xyth, fp_point2d, up_VMeCommand>*, std::string);
  ~JsonLogger();
  virtual void logPositionAndError() const noexcept;

};

class LoggerIsIncompatibleWithModelType : public std::exception {};

#endif  // VME_NMPC_SRC_LOGGERS_VIRTUALMELOGGER_HPP_