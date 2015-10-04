# set(warnings "-Wall -Wextra -Werror -pedantic-errors")
# set(optimizations "-flto")
# set(debug "-ggdb")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 ${warnings} ${optimizations} ${debug}")

file(GLOB Obstacle_sources Obstacle.cpp ObstacleTypes/*.cpp)
add_library (Obstacle ${Obstacle_sources})
add_library (NmpcEngine NmpcEngine.cpp NmpcModels/VirtualMeModel.cpp NmpcMinimizer.hpp NmpcMinimizers/VirtualMeSDMinimizer.cpp NmpcMinimizers/VirtualMeSDMinimizer.hpp)

# add_executable (vme-nmpc vme-nmpc.cpp)
# target_link_libraries (vme-nmpc Obstacle)