file(GLOB Obstacle_sources Obstacle.cpp ObstacleTypes/*.cpp)
add_library (Obstacle ${Obstacle_sources})
add_library (VMeNmpcEngine VMeNmpcEngine.cpp
                                 Subject.cpp
                                 VMeCommand.cpp
            )
add_library (Nav2Robot Nav2Robot.cpp)

# add_executable (vme-nmpc vme-nmpc.cpp)
# target_link_libraries (vme-nmpc Obstacle)