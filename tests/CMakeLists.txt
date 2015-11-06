macro (create_test target)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

add_library (CATCH catch.cpp)

set(testname utest-VMeNmpcEngine)
    add_executable (${testname} ${testname}.cpp
                        FakeVMeModel.cpp
                        FakeMinimizer.cpp
                        FakeExecutor.cpp
                        ../src/VMeCommand.cpp
                    )
    target_link_libraries (${testname} CATCH VMeNmpcEngine Nav2Robot)
    create_test(${testname})

set(testname utest-ObstacleContainer)
    add_executable (${testname} ${testname}.cpp
                    )
    target_link_libraries (${testname} CATCH Obstacle)
    create_test(${testname})

set(testname utest-VMeModel)
    add_executable (${testname} ${testname}.cpp)
    target_link_libraries (${testname} CATCH Obstacle VMeModel VMeNmpcInitPkg)
    create_test(${testname})

set(testname utest-JsonLogger)
    add_executable (${testname} ${testname}.cpp
                        FakeVMeModel.cpp
                        FakeExecutor.cpp
                        ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp
                        ../src/Loggers/JsonLogger.cpp
                    )
    target_link_libraries (${testname} CATCH VMeNmpcEngine VMeModel Obstacle VMeNmpcInitPkg)
    create_test(${testname})

set(testname utest-VMeNaiveSdMinimizer)
    add_executable (${testname} ${testname}.cpp
                        FakeVMeModel.cpp
                        ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp
                    )
    target_link_libraries (${testname} CATCH VMeModel Obstacle VMeNmpcInitPkg)
    create_test(${testname})

set(testname itest-VMe-Model_NaiveSdMinimizer)
    add_executable (${testname} ${testname}.cpp
                        ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp
                        ../src/Loggers/JsonLogger.cpp
                        FakeExecutor.cpp
                    )
    target_link_libraries (${testname} CATCH VMeNmpcEngine VMeModel Obstacle ${Boost_LIBRARIES})
    create_test(${testname})

# add_executable(non-catch-test non-catch-test.cpp FakeVMeModel.cpp ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp)
# target_link_libraries (non-catch-test VMeModel Obstacle)

#set(testname PathPlannerTest)
#add_executable (${testname} ${testname}.cpp)
#target_link_libraries (${testname} VMeNmpcEngine Obstacle)
#create_test(${testname})