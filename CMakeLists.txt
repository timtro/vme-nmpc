cmake_minimum_required (VERSION 2.8)
macro(listAllVariables)
get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()
endmacro()

#set(CMAKE_C_COMPILER /usr/bin/clang)
#set(CMAKE_CXX_COMPILER /usr/bin/clang++)

project(vme-nmpc)

# find_package(Boost 1.53.0 COMPONENTS log REQUIRED)
find_package (Threads)
# link_libraries(${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(c++std "-std=c++14")

set(warnings_debug "-Wall -Wextra -pedantic")
set(optimizations_debug "-O0")
set(debug "-g")

set(warnings_release "-Wall")
set(optimizations_releast "-O3 -flto -ffast-math")
set(perf_bench "-fno-omit-frame-pointer")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${c++std} ${debug} ${warnings_debug} ${optimizations_debug}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${c++std}")

# for clang++ use -fuse-ld=gold to avoid LD error "File format not recognized"
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  if(CMAKE_BUILD_TYPE MATCHES DEBUG)
    set(sanitizer "-fsanitize=address")
    # set(sanitizer "-fsanitize=memory")
    # set(sanitizer "-fsanitize=memory -fsanitize-memory-track-origins")
    # set(sanitizer "-fsanitize=thread")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fuse-ld=gold")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${sanitizer}")
  endif(CMAKE_BUILD_TYPE MATCHES DEBUG)
endif()

enable_testing()

add_subdirectory(src)
add_subdirectory(tests)
