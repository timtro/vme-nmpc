file(GLOB Obstacle_sources Obstacle.cpp ObstacleTypes/*.cpp)
add_library (Obstacle ${Obstacle_sources})
add_library (VirtualMeNmpcEngine VirtualMeNmpcEngine.cpp
                                 Subject.cpp
                                 VirtualMeCommand.cpp
            )
add_library (Nav2Robot Nav2Robot.cpp)

# add_executable (vme-nmpc vme-nmpc.cpp)
# target_link_libraries (vme-nmpc Obstacle)