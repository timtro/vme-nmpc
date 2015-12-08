macro (create_test target)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

add_library (CATCH catch.cpp)
add_library (fakes FakeVMeModel.cpp
    FakeMinimizer.cpp
    FakeVMeMinimizer.cpp
    FakeExecutor.cpp
    )

set(testname utest-VMeNmpcKernel)
    add_executable (${testname} ${testname}.cpp
                        ../src/VMeCommand.cpp
                    )
    target_link_libraries (${testname}
        CATCH VMeNmpcKernel Nav2Robot fakes AggregatorInitializer)
    create_test(${testname})

set(testname utest-ObstacleContainer)
    add_executable (${testname} ${testname}.cpp
                    )
    target_link_libraries (${testname} CATCH Obstacle)
    create_test(${testname})

set(testname utest-VMeModel)
    add_executable (${testname} ${testname}.cpp)
    target_link_libraries (${testname}
        CATCH VMeModel AggregatorInitializer fakes Obstacle)
    create_test(${testname})

set(testname utest-JsonLogger)
    add_executable (${testname} ${testname}.cpp
                        ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp
                        ../src/Loggers/JsonLogger.cpp
                    )
    target_link_libraries (${testname}
        CATCH VMeNmpcKernel VMeModel Obstacle AggregatorInitializer fakes)
    create_test(${testname})

set(testname utest-VMeNaiveSdMinimizer)
    add_executable (${testname} ${testname}.cpp
                        ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp
                        ../src/PathPlanners/VMePathPlanner.cpp
                        ../src/Target.cpp
                    )
    target_link_libraries (${testname}
        CATCH VMeModel Obstacle AggregatorInitializer fakes)
    create_test(${testname})

set(testname iutest-CliHandler-Daemon)
    add_executable (${testname} ${testname}.cpp
                        ../src/CliHandler.cpp
                        ../src/Daemon.cpp
                        ../src/Target.cpp
                    )
    target_link_libraries (${testname} CATCH Obstacle ${CMAKE_THREAD_LIBS_INIT})
    create_test(${testname})

set(testname itest-VMe-Model_NaiveSdMinimizer)
    add_executable (${testname} ${testname}.cpp
                        ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp
                        ../src/Loggers/JsonLogger.cpp
                        ../src/PathPlanners/VMePathPlanner.cpp
                        ../src/Target.cpp
                        ../src/SeedPackage.cpp
                    )
    target_link_libraries (${testname}
        CATCH VMeNmpcKernel VMeModel AggregatorInitializer Obstacle fakes)
    create_test(${testname})

# add_executable(non-catch-test non-catch-test.cpp
#                 ../src/NmpcMinimizers/VMeNaiveSdMinimizer.cpp
#                 ../src/Loggers/JsonLogger.cpp
#                 )
# target_link_libraries (non-catch-test
#     CATCH VMeNmpcKernel VMeModel AggregatorInitializer Obstacle fakes)

#set(testname PathPlannerTest)
#add_executable (${testname} ${testname}.cpp)
#target_link_libraries (${testname} VMeNmpcKernel Obstacle)
#create_test(${testname})