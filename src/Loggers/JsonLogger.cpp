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
#include "../NmpcModels/VMeModel.hpp"
#include "../NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include <cstdio>

using vMeModelType = NmpcModel<xyth, fp_point2d, up_VMeCommand>;

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

  logFile = std::make_unique<CFileContainer>(outputFilePath);
  fp_out = logFile->fd;
  init.bindIntoAggregator(this);
  fprintf(fp_out, "[\n");
}

JsonLogger::~JsonLogger() { fprintf(fp_out, "\n]\n"); }

// TODO: In GCC 5.3, change std::end() to std::cend();
template <typename T>
void jsonPrintArray(FILE *fd, T array) {
  auto iter = std::begin(array);
  for (;;) {
    fprintf(fd, "%f", *iter);
    if (++iter == std::end(array)) break;
    fprintf(fd, ",");
  }
}

void JsonLogger::logModelState() const noexcept {
  if (printedFirstObject) {
    fprintf(fp_out, ",\n");
  } else {
    printedFirstObject = true;
  }
  fprintf(fp_out, "{\n    ");

  fprintf(fp_out, "\"x\" : [");
  jsonPrintArray(fp_out, model->get_x());
  fprintf(fp_out, "],\n");

  fprintf(fp_out, "    \"y\" : [");
  jsonPrintArray(fp_out, model->get_y());
  fprintf(fp_out, "],\n");

  fprintf(fp_out, "    \"ex\" : [");
  jsonPrintArray(fp_out, model->get_ex());
  fprintf(fp_out, "],\n");

  fprintf(fp_out, "    \"ey\" : [");
  jsonPrintArray(fp_out, model->get_ey());
  fprintf(fp_out, "],\n");

  fprintf(fp_out, "    \"Dth\" : [");
  jsonPrintArray(fp_out, model->Dth);
  fprintf(fp_out, "]\n");

  fprintf(fp_out, "}");
  fflush(fp_out);
}

void JsonLogger::logMinimizerState() const noexcept {
  if (printedFirstObject) {
    fprintf(fp_out, ",\n");
  } else {
    printedFirstObject = true;
  }
  fprintf(fp_out, "{\n    ");
  fprintf(fp_out, "  \"iterations:\" : %d", minimizer->lastSdLoopCount);
  fprintf(fp_out, "}");
  fflush(fp_out);
}