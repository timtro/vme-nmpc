#include "catch.hpp"

#include "../src/Loggers/JsonLogger.hpp"
#include "../src/VMeNmpcKernel.hpp"
#include "../src/NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "FakeVMeModel.hpp"
#include "FakeVMeMinimizer.hpp"
#include "../src/CFileContainer.hpp"

using std::unique_ptr;
using std::make_unique;

struct TestObject {
  unsigned int nmpcHorizon = 10;
  float timeInterval = 0.1f;
  float speed = .4;

  AggregatorInitializer init;
  unique_ptr<VMeModel> model{nullptr};
  unique_ptr<VMeNaiveSdMinimizer> minimizer{nullptr};
  unique_ptr<JsonLogger> logger{nullptr};
  unique_ptr<VMeNmpcKernel> engine{nullptr};

  TestObject() {
    init.parameters->nmpcHorizon = nmpcHorizon;
    init.parameters->timeInterval = timeInterval;
    init.parameters->cruiseSpeed = speed;
    init.parameters->Q = 1;
    init.parameters->Q0 = init.parameters->Q / 2;
    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    logger = make_unique<JsonLogger>(init);
    engine = make_unique<VMeNmpcKernel>(init);
  }

  TestObject(std::string logFilePath) {
    init.parameters->nmpcHorizon = nmpcHorizon;
    init.parameters->timeInterval = timeInterval;
    init.parameters->cruiseSpeed = speed;
    init.parameters->Q = 1;
    init.parameters->Q0 = init.parameters->Q / 2;
    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    logger = make_unique<JsonLogger>(init, logFilePath);
    engine = make_unique<VMeNmpcKernel>(init);
  }
};

TEST_CASE(
    "Throw LoggerIsIncompatibleWithModelType if I try to pass an unfamilliar "
    "model to the logger initializer") {
  std::string notUsed;
  AggregatorInitializer init;
  init.parameters->nmpcHorizon = 5;
  // Must name these since scope only applies to named objects.
  auto tmpModel = make_unique<FakeVMeModel>(init, notUsed);
  auto tmpMinimizer = make_unique<FakeVMeMinimizer>(init, notUsed);
  REQUIRE_THROWS_AS(make_unique<JsonLogger>(init),
                    LoggerIsIncompatibleWithModelType);
}

TEST_CASE("Straightforward write to stdout with nothing to assert.") {
  TestObject test;
  test.engine->seed(xyth{0, 0, 0});
}

// TEST_CASE("Logger write to file. TODO: Assert against file contents.") {
//   TestObject test("loggertest.log");
//   test.engine->seed(xyth{0, 0, 0}, fp_point2d{5, 0});
// }