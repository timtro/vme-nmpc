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
#include <cstdio>
#include "../AggregatorInitializer.hpp"
#include "../NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "../NmpcModels/VMeModel.hpp"
#include "../Target.hpp"

// Add new obstacle types here:
#include "../ObstacleTypes/PointObstacle.hpp"

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
  init.logger_binding_safety_check();
  this->model = guranteedCompatibleModel(init.model);
  this->minimizer = guranteedCompatibleMminimizer(init.minimizer);
  init.bind_into_aggregator(this);
  fprintf(fp_out, "[\n");
}

JsonLogger::JsonLogger(AggregatorInitializer &init, FILE *outputFilePtr)
    : fp_out{outputFilePtr} {
  init.logger_binding_safety_check();
  this->model = guranteedCompatibleModel(init.model);
  this->minimizer = guranteedCompatibleMminimizer(init.minimizer);
  init.bind_into_aggregator(this);
  fprintf(fp_out, "[\n");
}

JsonLogger::JsonLogger(AggregatorInitializer &init,
                       std::string outputFilePath) {
  init.logger_binding_safety_check();
  this->model = guranteedCompatibleModel(init.model);
  this->minimizer = guranteedCompatibleMminimizer(init.minimizer);

  logFile = std::make_unique<CFileContainer>(outputFilePath, "w");
  fp_out = logFile->fd;
  init.bind_into_aggregator(this);
  fprintf(fp_out, "[\n");
}

JsonLogger::~JsonLogger() { fprintf(fp_out, "\n]\n"); }

void jsonPrintOpenObject(FILE *);
void jsonPrintCloseObject(FILE *);
template <typename T>
void jsonPrintArrayNode(FILE *fd, std::string, const T);
template <typename T>
void jsonPrintArray(FILE *, const T &);

void JsonLogger::log_constants(const AggregatorInitializer &init) const
    noexcept {
  if (printedFirstObject) {
    fprintf(fp_out, ",\n");
  } else {
    printedFirstObject = true;
  }
  fprintf(fp_out, "{\n");
  fprintf(fp_out, "  \"nmpcHorizon\": %d,\n",
          static_cast<int>(init.get_nmpcHorizon()));
  fprintf(fp_out, "  \"timeInterval\" : %f,\n", init.get_timeInterval());
  fprintf(fp_out, "  \"cruiseSpeed\" : %f,\n", init.get_cruiseSpeed());
  fprintf(fp_out, "  \"R\" : %f,\n", init.get_R());
  fprintf(fp_out, "  \"Q\" : %f,\n", init.get_Q());
  fprintf(fp_out, "  \"Q0\" : %f,\n", init.get_Q0());
  fprintf(fp_out, "  \"sdStepFactor\" : %f,\n", init.get_sdStepFactor());
  fprintf(fp_out, "  \"sdConvergenceTolerance\" : %f,\n",
          init.get_sdConvergenceTolerance());
  fprintf(fp_out, "  \"maxSdSteps\" : %d,\n",
          static_cast<int>(init.get_maxSdSteps()));
  fprintf(fp_out, "  \"jsonLogPath\" : \"%s\"\n",
          init.get_jsonLogPath().c_str());
  fprintf(fp_out, "},\n");
  fflush(fp_out);
}

void JsonLogger::log_model_state() const noexcept {
  fprintf(fp_out, "{\n");
  jsonPrintArrayNode(fp_out, "x", model->get_x());
  jsonPrintArrayNode(fp_out, "y", model->get_y());
  jsonPrintArrayNode(fp_out, "ex", model->get_ex());
  jsonPrintArrayNode(fp_out, "ey", model->get_ey());
  jsonPrintArrayNode(fp_out, "Dth", model->Dth);
  fprintf(fp_out, "},\n");
  fflush(fp_out);
}

void JsonLogger::log_minimizer_state() const noexcept {
  fprintf(fp_out, "{");
  fprintf(fp_out, "  \"iterations\" : %d ", minimizer->lastSdLoopCount);
  fprintf(fp_out, "},\n");
  fflush(fp_out);
}

bool is_unknown_obstacle_type(const std::unique_ptr<Obstacle> &);
void JsonLogger::log_obstacles(const ObstacleContainer &obstacles) const
    noexcept {
  for (auto const &obstacle : obstacles) {
    if (is_unknown_obstacle_type(obstacle)) printf("HELLO\n");
  }
}

void JsonLogger::log_targets(const TargetContainer &targets) const noexcept {
  fprintf(fp_out, "{\n");
  fprintf(fp_out, "  \"targets\" : [");
  auto iter = std::begin(targets);
  for (;;) {
    fprintf(fp_out, "[%f, %f, %f]", (*iter)->x, (*iter)->y, (*iter)->tolerance);
    if (++iter == std::end(targets)) break;
    fprintf(fp_out, ",");
  }
  fprintf(fp_out, "]\n},\n");
  fflush(fp_out);
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

bool is_unknown_obstacle_type(const std::unique_ptr<Obstacle> &obstacle) {
  return true;
}