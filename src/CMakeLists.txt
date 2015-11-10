
file(GLOB Obstacle_sources Obstacle.cpp ObstacleTypes/*.cpp)
add_library (Obstacle ${Obstacle_sources})

add_library (VMeNmpcEngine VMeNmpcEngine.cpp
                                 Subject.cpp
            )

add_library (Nav2Robot Nav2Robot.cpp)

add_library (VMeModel NmpcModels/VMeModel.cpp VMeCommand.cpp)
  target_link_libraries(VMeModel Nav2Robot)

add_library (VMeNmpcInitPkg VMeNmpcInitPkg.cpp)
  target_link_libraries(VMeNmpcInitPkg VMeModel)

add_executable (vme-nmpc vme-nmpc.cpp ClArgs.cpp InputFileData.cpp)
target_link_libraries (vme-nmpc Obstacle VMeNmpcEngine VMeModel)