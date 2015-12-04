
file(GLOB Obstacle_sources Obstacle.cpp ObstacleTypes/*.cpp)
add_library (Obstacle ${Obstacle_sources})

add_library (VMeNmpcKernel VMeNmpcKernel.cpp
                                 Subject.cpp
            )

add_library (Nav2Robot Nav2Robot.cpp)

add_library (VMeModel NmpcModels/VMeModel.cpp VMeCommand.cpp SeedPackage.cpp)
  target_link_libraries(VMeModel Nav2Robot)

add_library (AggregatorInitializer AggregatorInitializer.cpp)
  target_link_libraries(AggregatorInitializer VMeModel)

add_executable (vme-nmpc vme-nmpc.cpp
                  ClArgs.cpp
                  InputFileData.cpp
                  Daemon.cpp
                  NmpcMinimizers/VMeNaiveSdMinimizer.cpp
                  Loggers/JsonLogger.cpp
                  AggregatorInitializer.cpp
                  CliHandler.cpp
                )
target_link_libraries (vme-nmpc VMeNmpcKernel VMeModel Obstacle ${CMAKE_THREAD_LIBS_INIT})
set_source_files_properties(vme-nmpc.cpp PROPERTIES COMPILE_FLAGS -DBOOST_LOG_DYN_LINK)