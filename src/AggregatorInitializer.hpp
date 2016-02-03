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

#ifndef VME_NMPC_NMPCINITPKG_HPP_
#define VME_NMPC_NMPCINITPKG_HPP_

#include <memory>

#include "typedefs.h"
#include "linear.hpp"
#include "NmpcModel.hpp"
#include "NmpcMinimizer.hpp"
#include "PathPlanner.hpp"
#include "VMeLogger.hpp"
#include "VMeCommand.hpp"
#include "InputFileData.hpp"
#include "Target.hpp"
#include "PathPlanner.hpp"
#include "SeedPackage.hpp"

class ObstacleContainer;

using vMeModelType = NmpcModel<SeedPackage, up_VMeCommand>;

/*
 * This class provides a convenient means to handle a complex dependency
 *  injection.
 *   An AggregatorInitializer is used to initialize the model, minimizer, logger
 * and kernel by passing it as an argument to their constructors. It retains
 * pointers to each of the objects it initializes so that when it is used to
 * initialize aggregate objects, those objects can take the pointers from the
 * AggregatorInitializer. Therefore, objects which are initialized with a
 * particular AggregatorInitializer will hang together. A programmer
 * building an NMPC kernel need only make sure to initialize things in the
 * right order.
 */
struct AggregatorInitializer {
  AggregatorInitializer(const AggregatorInitializer&) = delete;
  AggregatorInitializer& operator=(const AggregatorInitializer&) = delete;
  AggregatorInitializer();
  AggregatorInitializer(InputFileData&);

  InputFileData* parameters;
  vMeModelType* model{nullptr};
  NmpcMinimizer* minimizer{nullptr};
  VMeLogger* logger{nullptr};
  PathPlanner<SeedPackage>* planner{nullptr};
  ObstacleContainer* obstacles{nullptr};
  TargetContainer* targets{nullptr};

  void modelBindingSafetyCheck();
  void minimizerBindingSafetyCheck();
  void loggerBindingSafetyCheck();
  void aggregatorCompletionSafetyCheck();
  void bindIntoAggregator(vMeModelType*);
  void bindIntoAggregator(NmpcMinimizer*);
  void bindIntoAggregator(VMeLogger*);
  void bindIntoAggregator(PathPlanner<SeedPackage>*);

  unsigned get_nmpcHorizon() const;
  fptype get_timeInterval() const;
  fptype get_cruiseSpeed() const;
  fptype get_Q() const;
  fptype get_Q0() const;
  fptype get_R() const;
  fptype get_sdStepFactor() const;
  fptype get_sdConvergenceTolerance() const;
  unsigned get_maxSdSteps() const;
  fptype get_targetDistanceTolerance() const;
  std::string get_jsonLogPath() const;

 private:
  // only for default construction:
  std::unique_ptr<InputFileData> defaultParameters_;
};

class ModelMustBeInitializedBeforeMinimizerOrLogger : public std::exception {
  virtual const char* what() const noexcept override {
    return "A model must be initialized with the AggregatorInitializer before "
           "a minimizer.";
  }
};

class InitPkgCanOnlyBeUsedOnceToInitializeAModel : public std::exception {
  virtual const char* what() const noexcept override {
    return "An attempt was made to initialize a model with an "
           "AggregatorInitializer that is already bound to one.";
  }
};

class InitPkgDoesNotContainPointerToAModel : public std::exception {
  virtual const char* what() const noexcept override {
    return "AggregatorInitializer contains no pointer to model at a time when "
           "it should.";
  }
};

class InitPkgDoesNotContainPointerToAMinimizer : public std::exception {
  virtual const char* what() const noexcept override {
    return "AggregatorInitializer contains no pointer to a minimizer at a time "
           "when it should.";
  }
};

class InitPkgAlreadyHasBoundMinimizer : public std::exception {
  virtual const char* what() const noexcept override {
    return "An attempt was made to initialize a minimizer with an "
           "AggregatorInitializer that is already bound to one.";
  }
};

class InitPkgDoesNotContainPointerToAPathPlanner : public std::exception {
  virtual const char* what() const noexcept override {
    return "AggregatorInitializer contains no PathPlanner at finalization.";
  }
};

#endif  // VME_NMPC_NMPCINITPKG_HPP_
