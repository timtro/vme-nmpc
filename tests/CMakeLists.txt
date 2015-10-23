macro (create_test target)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

add_library (CATCH catch.cpp)

set(testname VirtualMeNmpcEngineTest)
add_executable (${testname} ${testname}.cpp
                            FakeVirtualMeModel.cpp
                            FakeMinimizer.cpp
                            ../src/Nav2Robot.cpp
                )
target_link_libraries (${testname} CATCH VirtualMeNmpcEngine)
create_test(${testname})

set(testname ObstacleContainerTest)
add_executable (${testname} ${testname}.cpp
                )
target_link_libraries (${testname} CATCH Obstacle)
create_test(${testname})

set(testname VirtualMeModelTest)
add_executable (${testname} ${testname}.cpp
                            ../src/NmpcModels/VirtualMeModel.cpp
                            ../src/VirtualMeCommand.cpp
                )
target_link_libraries (${testname} CATCH Obstacle Nav2Robot)
create_test(${testname})

#set(testname PathPlannerTest)
#add_executable (${testname} ${testname}.cpp)
#target_link_libraries (${testname} VirtualMeNmpcEngine Obstacle)
#create_test(${testname})