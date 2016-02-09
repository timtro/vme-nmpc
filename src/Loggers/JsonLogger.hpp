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

#include "../CFileContainer.hpp"
#include "../VMeLogger.hpp"

struct AggregatorInitializer;
class VMeModel;
class VMeNaiveSdMinimizer;

/**
 * Logger writes a simple text file in JSON format:
 *     http://www.json.org/
 */
class JsonLogger : public VMeLogger {
  VMeModel* model{nullptr};
  VMeNaiveSdMinimizer* minimizer{nullptr};
  FILE* fp_out{stdout};
  std::unique_ptr<CFileContainer> logFile;
  mutable bool printedFirstObject{false};

 public:
  JsonLogger(const JsonLogger&) = delete;
  JsonLogger& operator=(const JsonLogger&) = delete;
  JsonLogger(AggregatorInitializer&);
  JsonLogger(AggregatorInitializer&, FILE*);
  JsonLogger(AggregatorInitializer&, std::string);
  ~JsonLogger();

  virtual void log_model_state() const noexcept;
  virtual void log_minimizer_state() const noexcept;
  virtual void log_constants(const AggregatorInitializer&) const noexcept;
};

class LoggerIsIncompatibleWithModelType : public std::exception {
  virtual const char* what() const noexcept override {
    return "The logger is not familliar with the model type.";
  }
};

class LoggerIsIncompatibleWithMinimizerType : public std::exception {
  virtual const char* what() const noexcept override {
    return "The logger is not familliar with the minimizer type.";
  }
};

#endif  // VME_NMPC_SRC_LOGGERS_VIRTUALMELOGGER_HPP_
