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

#include "InputFileData.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>

namespace pt = boost::property_tree;

void InputFileData::load(const std::string& filename) {
  pt::ptree tree;
  pt::read_json(filename, tree);

  N = tree.get<int>("N");
  m = tree.get<int>("m");
  n = tree.get<int>("n");
  T = tree.get<float>("T");
  dg = tree.get<float>("dg");
  tgttol = tree.get<float>("tgttol");
  cruising_speed = tree.get<float>("cruising_speed");
  Q = tree.get<float>("Q");
  Q0 = tree.get<float>("Q0");
  R = tree.get<float>("R");
}