cmake_minimum_required (VERSION 2.8)

set(CMAKE_C_COMPILER /usr/bin/clang)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)

project(vme-nmpc)
set(CMAKE_BUILD_TYPE debug)


set(bld_dir ${CMAKE_CURRENT_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${bld_dir} CACHE PATH "Build directory" FORCE)
set(LIBRARY_OUTPUT_PATH ${bld_dir} CACHE PATH "Build directory" FORCE)

set(warnings "-Wall -Wextra -pedantic")
set(optimizations "-flto")
set(debug "-ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 -fuse-ld=gold ${warnings} ${optimizations} ${debug}")

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)