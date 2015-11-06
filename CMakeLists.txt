cmake_minimum_required (VERSION 3.3)
macro(listAllVariables)
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
endmacro()

set(CMAKE_C_COMPILER /usr/bin/clang)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)

project(vme-nmpc)
set(CMAKE_BUILD_TYPE debug)

# find_package(Boost 1.53.0 COMPONENTS log REQUIRED)
# find_package (Threads)
# link_libraries(${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


# set(bld_dir ${CMAKE_CURRENT_SOURCE_DIR}/build)
# set(EXECUTABLE_OUTPUT_PATH ${bld_dir} CACHE PATH "Build directory" FORCE)
# set(LIBRARY_OUTPUT_PATH ${bld_dir} CACHE PATH "Build directory" FORCE)

set(warnings "-Wall -Wextra -pedantic")
set(optimizations "-flto") #for perf: -fno-omit-frame-pointer
set(debug "-g")
#-ffast-math -O3

# for clang++ use -fuse-ld=gold to avoid LD error "File format not recognized"
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold") # -fsanitize=address conflicts with CATCH
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 ${warnings} ${optimizations} ${debug}")

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)
