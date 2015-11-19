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

#ifndef VME_NMPC_SRC_FDRAIIWRAPPER_HPP_
#define VME_NMPC_SRC_FDRAIIWRAPPER_HPP_

#include <stdexcept>
#include <string>
#include <cstdio>

class ErrorOpeningFileWithFopen : public std::runtime_error {
 public:
  ErrorOpeningFileWithFopen(std::string const& msg) : std::runtime_error(msg) {}
};

struct CFileContainer {
  FILE* fd;
  CFileContainer(const CFileContainer&) = delete;
  CFileContainer& operator=(const CFileContainer&) = delete;
  CFileContainer(std::string filepath, std::string rw) {
    fd = fopen(filepath.c_str(), rw.c_str());
    if (fd == nullptr)
      throw ErrorOpeningFileWithFopen(std::string{"File path: '"} + filepath +
                                      "'.");
  }
  ~CFileContainer() { fclose(fd); }
};

#endif  // VME_NMPC_SRC_FDRAIIWRAPPER_HPP_