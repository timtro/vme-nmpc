cmake_minimum_required (VERSION 2.8)

set(dir ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)

project (vme-nmpc)

set(CMAKE_BUILD_TYPE Debug)

INCLUDE(FindPkgConfig)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# The version number.
set (vme-nmpc_VERSION_MAJOR 2)
set (vme-nmpc_VERSION_MINOR 0)

# configure a header file to pass some of the CMake settings
# to the source code
#configure_file (
#  "${PROJECT_SOURCE_DIR}/vme-nmpcConfig.h.in"
#  "${PROJECT_BINARY_DIR}/vme-nmpcConfig.h"
#  )

# add the binary tree to the search path for include files
# so that we will find vme-nmpcConfig.h
#include_directories("${PROJECT_BINARY_DIR}")

#set(SRCS main.cpp init.cpp)

FILE(GLOB vme-nmpc-sources src/*.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -Wall -pedantic-errors -flto -lpthread -std=c++14")

# add the executable
add_executable(vme-nmpc ${vme-nmpc-sources})

# find_package (HDF5)
# if (HDF5_FOUND)
#   include_directories(${HDF5_INCLUDE_DIRS})
#   target_link_libraries (vme-nmpc ${HDF5_LIBRARIES})
# endif (HDF5_FOUND)