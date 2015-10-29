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

#include "StdoutJsonLogger.hpp"
#include "../NmpcModels/VirtualMeModel.hpp"
#include <cstdio>

StdoutJsonLogger::StdoutJsonLogger(
    NmpcModel<xyvth, fp_point2d, up_VirtualMeCommand>* model) {
  auto modelToLog = dynamic_cast<VirtualMeModel*>(model);
  if (modelToLog == nullptr) throw LoggerIsIncompatibleWithModelType();
  this->model = modelToLog;
  fprintf(fp_out, "[\n");
}

StdoutJsonLogger::StdoutJsonLogger(
    NmpcModel<xyvth, fp_point2d, up_VirtualMeCommand>* model,
    FILE* outputFilePtr)
    : fp_out{outputFilePtr} {
  auto modelToLog = dynamic_cast<VirtualMeModel*>(model);
  if (modelToLog == nullptr) throw LoggerIsIncompatibleWithModelType();
  this->model = modelToLog;
  fprintf(fp_out, "[\n");
}

StdoutJsonLogger::StdoutJsonLogger(
    NmpcModel<xyvth, fp_point2d, up_VirtualMeCommand>* model,
    std::string outputFilePath) {
  auto modelToLog = dynamic_cast<VirtualMeModel*>(model);
  if (modelToLog == nullptr) throw LoggerIsIncompatibleWithModelType();
  this->model = modelToLog;
  logFile = std::make_unique<CFileContainer>(outputFilePath);
  fp_out = logFile->fd;
  fprintf(fp_out, "[\n");
}

StdoutJsonLogger::~StdoutJsonLogger() { fprintf(fp_out, "\n]\n"); }

// TODO: In GCC 5.3, change std::end() to std::cend();
template <typename T>
void jsonPrintArray(FILE* fd, T array) {
  auto iter = std::begin(array);
  for (;;) {
    fprintf(fd, "%f", *iter);
    if (++iter == std::end(array)) break;
    fprintf(fd, ",");
  }
}

void StdoutJsonLogger::logPositionAndError() const noexcept {
  if (printedFirstObject) {
    fprintf(fp_out, ",\n");
  } else {
    printedFirstObject = true;
  }
  fprintf(fp_out, "{\n    ");
  fprintf(fp_out, "\"x\" : [");
  jsonPrintArray(fp_out, model->getX());
  fprintf(fp_out, "],\n");
  fprintf(fp_out, "    \"y\" : [");
  jsonPrintArray(fp_out, model->getY());
  fprintf(fp_out, "],\n");
  fprintf(fp_out, "    \"Ex\" : [");
  jsonPrintArray(fp_out, model->getEx());
  fprintf(fp_out, "],\n");
  fprintf(fp_out, "    \"Ey\" : [");
  jsonPrintArray(fp_out, model->getEy());
  fprintf(fp_out, "]\n");
  fprintf(fp_out, "}");
  fflush(fp_out);
}