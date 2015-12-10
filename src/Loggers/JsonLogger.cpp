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

#include "JsonLogger.hpp"
#include "../AggregatorInitializer.hpp"
#include "../NmpcModels/VMeModel.hpp"
#include "../NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include <cstdio>

using vMeModelType = NmpcModel<SeedPackage, up_VMeCommand>;

auto guranteedCompatibleModel(vMeModelType *model) {
  auto modelToBeLogged = dynamic_cast<VMeModel *>(model);
  if (modelToBeLogged == nullptr) throw LoggerIsIncompatibleWithModelType();
  return modelToBeLogged;
}

auto guranteedCompatibleMminimizer(NmpcMinimizer *model) {
  auto minimizerToBeLogged = dynamic_cast<VMeNaiveSdMinimizer *>(model);
  if (minimizerToBeLogged == nullptr)
    throw LoggerIsIncompatibleWithMinimizerType();
  return minimizerToBeLogged;
}

JsonLogger::JsonLogger(AggregatorInitializer &init) {
  init.loggerBindingSafetyCheck();
  this->model = guranteedCompatibleModel(init.model);
  this->minimizer = guranteedCompatibleMminimizer(init.minimizer);
  init.bindIntoAggregator(this);
  fprintf(fp_out, "[\n");
}

JsonLogger::JsonLogger(AggregatorInitializer &init, FILE *outputFilePtr)
    : fp_out{outputFilePtr} {
  init.loggerBindingSafetyCheck();
  this->model = guranteedCompatibleModel(init.model);
  this->minimizer = guranteedCompatibleMminimizer(init.minimizer);
  init.bindIntoAggregator(this);
  fprintf(fp_out, "[\n");
}

JsonLogger::JsonLogger(AggregatorInitializer &init,
                       std::string outputFilePath) {
  init.loggerBindingSafetyCheck();
  this->model = guranteedCompatibleModel(init.model);
  this->minimizer = guranteedCompatibleMminimizer(init.minimizer);

  logFile = std::make_unique<CFileContainer>(outputFilePath, "w");
  fp_out = logFile->fd;
  init.bindIntoAggregator(this);
  fprintf(fp_out, "[\n");
}

JsonLogger::~JsonLogger() { fprintf(fp_out, "\n]\n"); }

void jsonPrintOpenObject(FILE *);
void jsonPrintCloseObject(FILE *);
void jsonPrintNode(FILE *, std::string, std::string);
void jsonPrintNode(FILE *, std::string, int);
void jsonPrintNode(FILE *, std::string, float);
template <typename T>
void jsonPrintArrayNode(FILE *fd, std::string, const T);
template <typename T>
void jsonPrintArray(FILE *, const T &);

void JsonLogger::logConstants(const AggregatorInitializer &init) const
    noexcept {
  if (printedFirstObject) {
    fprintf(fp_out, ",\n");
  } else {
    printedFirstObject = true;
  }
  jsonPrintOpenObject(fp_out);
  jsonPrintNode(fp_out, "nmpcHorizon:",
                static_cast<int>(init.get_nmpcHorizon()));
  jsonPrintNode(fp_out, "timeInterval", init.get_timeInterval());
  jsonPrintNode(fp_out, "cruiseSpeed", init.get_cruiseSpeed());
  jsonPrintNode(fp_out, "R", init.get_R());
  jsonPrintNode(fp_out, "Q", init.get_Q());
  jsonPrintNode(fp_out, "Q0", init.get_Q0());
  jsonPrintNode(fp_out, "sdStepFactor", init.get_sdStepFactor());
  jsonPrintNode(fp_out, "sdConvergenceTolerance",
                init.get_sdConvergenceTolerance());
  jsonPrintNode(fp_out, "maxSdSteps", static_cast<int>(init.get_maxSdSteps()));
  jsonPrintNode(fp_out, "jsonLogPath", init.get_jsonLogPath());
  jsonPrintCloseObject(fp_out);
}

void JsonLogger::logModelState() const noexcept {
  if (printedFirstObject) {
    fprintf(fp_out, ",\n");
  } else {
    printedFirstObject = true;
  }
  jsonPrintOpenObject(fp_out);
  jsonPrintArrayNode(fp_out, "x", model->get_x());
  jsonPrintArrayNode(fp_out, "y", model->get_y());
  jsonPrintArrayNode(fp_out, "ex", model->get_ex());
  jsonPrintArrayNode(fp_out, "ey", model->get_ey());
  jsonPrintArrayNode(fp_out, "Dth", model->Dth);
  jsonPrintCloseObject(fp_out);
}

void JsonLogger::logMinimizerState() const noexcept {
  if (printedFirstObject) {
    fprintf(fp_out, ",\n");
  } else {
    printedFirstObject = true;
  }
  jsonPrintOpenObject(fp_out);
  jsonPrintNode(fp_out, "iterations", minimizer->lastSdLoopCount);
  jsonPrintCloseObject(fp_out);
}

void jsonPrintOpenObject(FILE *fd) { fprintf(fd, "{\n"); }

void jsonPrintCloseObject(FILE *fd) {
  fprintf(fd, "}");
  fflush(fd);
}

void jsonPrintNode(FILE *fd, std::string name, std::string data) {
  fprintf(fd, "  \"%s\" : %s\n", name.c_str(), data.c_str());
}

void jsonPrintNode(FILE *fd, std::string name, int data) {
  fprintf(fd, "  \"%s\" : %d\n", name.c_str(), data);
}

void jsonPrintNode(FILE *fd, std::string name, float data) {
  fprintf(fd, "  \"%s\" : %f\n", name.c_str(), data);
}

template <typename T>
void jsonPrintArrayNode(FILE *fd, std::string name, const T array) {
  fprintf(fd, "  \"%s\" : [", name.c_str());
  jsonPrintArray(fd, std::forward<const T>(array));
  fprintf(fd, "],\n");
}

// In GCC < 5.3, change std::cend() to std::end();
template <typename T>
void jsonPrintArray(FILE *fd, const T &array) {
  auto iter = std::begin(array);
  for (;;) {
    fprintf(fd, "%f", *iter);
    if (++iter == std::end(array)) break;
    fprintf(fd, ",");
  }
}