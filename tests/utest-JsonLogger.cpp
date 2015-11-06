#include "catch.hpp"

#include "../src/Loggers/JsonLogger.hpp"
#include "../src/VMeNmpcEngine.hpp"
#include "../src/NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "FakeVMeModel.hpp"
#include "../src/CFileContainer.hpp"

struct TestObject {
  VMeNmpcEngine* eng{nullptr};
  unsigned int nmpcHorizon = 10;
  float timeInterval = 0.1f;
  float speed = .4;
  VMeNmpcInitPkg init;

  TestObject() {
    init.horizonSize = nmpcHorizon;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
    new VMeModel{init};
    new VMeNaiveSdMinimizer{init};
    new JsonLogger{init};
    eng = new VMeNmpcEngine{init};
  }

  TestObject(std::string logFilePath) {
    init.horizonSize = nmpcHorizon;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
    new VMeModel{init};
    new VMeNaiveSdMinimizer{init};
    new JsonLogger{init, logFilePath};
    eng = new VMeNmpcEngine{init};
  }
  ~TestObject() { delete eng; }
  auto* model() { return eng->_getModelPointer_(); }
  auto* minimizer() { return eng->_getMinimizerPointer_(); }
};

TEST_CASE(
    "Throw LoggerIsIncompatibleWithModelType if I try to pass an unfamilliar "
    "model to the logger initializer") {
  std::string notUsed;
  VMeNmpcInitPkg init;
  init.horizonSize = 5;
  new FakeVMeModel(init, notUsed);

  REQUIRE_THROWS_AS(new JsonLogger(init), LoggerIsIncompatibleWithModelType);
}

TEST_CASE("Straightforward write to stdout with nothing to assert.") {
  TestObject test;
  test.eng->seed(xyth{0, 0, 0}, fp_point2d{5, 0});
}

// TEST_CASE("Logger write to file. TODO: Assert against file contents.") {
//   TestObject test("loggertest.log");
//   test.eng->seed(xyth{0, 0, 0}, fp_point2d{5, 0});
// }