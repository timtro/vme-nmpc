macro (create_test target)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

set(testname ObstacleTest)
add_executable (${testname} ${testname}.cpp)
target_link_libraries (${testname} Obstacle)
create_test(${testname})

set(testname VirtualMeModelTest)
add_executable (${testname} ${testname}.cpp)
target_link_libraries (${testname} NmpcEngine Obstacle)
create_test(${testname})

set(testname NmpcEngineTest)
add_executable (${testname} ${testname}.cpp ../src/NmpcMinimizer.hpp ../src/NmpcMinimizers/VirtualMeSDMinimizer.cpp ../src/NmpcMinimizers/VirtualMeSDMinimizer.hpp)
target_link_libraries (${testname} NmpcEngine Obstacle)
create_test(${testname})