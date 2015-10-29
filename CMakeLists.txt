cmake_minimum_required (VERSION 2.8)
macro(listAllVariables)
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
endmacro()

# set(CMAKE_C_COMPILER /usr/bin/clang)
# set(CMAKE_CXX_COMPILER /usr/bin/clang++)

project(vme-nmpc)
set(CMAKE_BUILD_TYPE debug)

# set(bld_dir ${CMAKE_CURRENT_SOURCE_DIR}/build)
# set(EXECUTABLE_OUTPUT_PATH ${bld_dir} CACHE PATH "Build directory" FORCE)
# set(LIBRARY_OUTPUT_PATH ${bld_dir} CACHE PATH "Build directory" FORCE)

set(warnings "-Wall -Wextra -pedantic")
# set(optimizations "-flto -ffast-math") #for perf: -fno-omit-frame-pointer
# set(optimizations "-fno-omit-frame-pointer -O0") #for perf: -fno-omit-frame-pointer
set(debug "-g")

# for clang++ use -fuse-ld=gold to avoid LD error "File format not recognized"
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 ${warnings} ${optimizations} ${debug}")

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)