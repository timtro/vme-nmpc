macro (create_test target)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

set(testname ObstacleTest)
add_executable (${testname} ${testname}.cpp)
target_link_libraries (${testname} Obstacle)
create_test(${testname})

set(testname NmpcModelTest)
add_executable (${testname} ${testname}.cpp)
target_link_libraries (${testname} NmpcModel)
create_test(${testname})