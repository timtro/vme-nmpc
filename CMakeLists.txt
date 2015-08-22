cmake_minimum_required (VERSION 2.8)

project (vme-nmpc)

set(dir ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)

set(CMAKE_BUILD_TYPE Debug)

include(FindPkgConfig)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# The version number.
set (vme-nmpc_VERSION_MAJOR 0)
set (vme-nmpc_VERSION_MINOR 8)

file(GLOB vme-nmpc-sources src/*.cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -ggdb -Wall -pedantic-errors -flto")

add_executable(vme-nmpc ${vme-nmpc-sources})

find_package (Threads)
if(Threads_FOUND)
  target_link_libraries(vme-nmpc -lpthread)
endif()

find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  target_link_libraries(vme-nmpc ${Boost_LIBRARIES})
endif()

# find_package (HDF5)
# if (HDF5_FOUND)
#   include_directories(${HDF5_INCLUDE_DIRS})
#   target_link_libraries (vme-nmpc ${HDF5_LIBRARIES})
# endif (HDF5_FOUND)