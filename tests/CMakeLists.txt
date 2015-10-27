macro (create_test target)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

add_library (CATCH catch.cpp)

set(testname utest-VirtualMeNmpcEngine)
add_executable (${testname} ${testname}.cpp
                            FakeVirtualMeModel.cpp
                            FakeMinimizer.cpp
                            FakeExecutor.cpp
                            ../src/Nav2Robot.cpp
                )
target_link_libraries (${testname} CATCH VirtualMeNmpcEngine)
create_test(${testname})

set(testname utest-ObstacleContainer)
add_executable (${testname} ${testname}.cpp
                )
target_link_libraries (${testname} CATCH Obstacle)
create_test(${testname})

set(testname utest-VirtualMeModel)
add_executable (${testname} ${testname}.cpp
                            ../src/NmpcModels/VirtualMeModel.cpp
                            ../src/VirtualMeCommand.cpp
                )
target_link_libraries (${testname} CATCH Obstacle Nav2Robot)
create_test(${testname})

set(testname itest-virtualME_Model-SDmin)
add_executable (${testname} ${testname}.cpp
                ../src/NmpcModels/VirtualMeModel.cpp
                ../src/NmpcMinimizers/VirtualMeSDMinimizer.cpp
                )
target_link_libraries (${testname} CATCH VirtualMeNmpcEngine Obstacle Nav2Robot)

set(testname utest-StdoutJsonLogger)
add_executable (${testname} ${testname}.cpp
                            FakeVirtualMeModel.cpp
                            FakeExecutor.cpp
                            ../src/NmpcModels/VirtualMeModel.cpp
                            ../src/NmpcMinimizers/VirtualMeSDMinimizer.cpp
                            ../src/Nav2Robot.cpp
                            ../src/Loggers/StdoutJsonLogger.cpp
                )
target_link_libraries (${testname} CATCH VirtualMeNmpcEngine Obstacle)
create_test(${testname})
#set(testname PathPlannerTest)
#add_executable (${testname} ${testname}.cpp)
#target_link_libraries (${testname} VirtualMeNmpcEngine Obstacle)
#create_test(${testname})