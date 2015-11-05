#include "catch.hpp"

#include "../src/Loggers/JsonLogger.hpp"
#include "../src/VMeNmpcEngine.hpp"
#include "../src/NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "FakeVMeModel.hpp"
#include "../src/CFileContainer.hpp"

struct standardTestSetup {
  VMeNmpcEngine* eng{nullptr};
  unsigned int nmpcHorizon = 10;
  float timeInterval = 0.1f;
  float speed = .4;
  VMeNmpcInitPkg init;

  standardTestSetup() {
    init.horizonSize = nmpcHorizon;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
    std::unique_ptr<VMeModel> mod{new VMeModel{init}};
    std::unique_ptr<VMeNaiveSdMinimizer> min{
        new VMeNaiveSdMinimizer{mod.get()}};
    std::unique_ptr<JsonLogger> logger{new JsonLogger(mod.get())};
    eng = new VMeNmpcEngine{std::move(mod), std::move(min),
                                  std::move(logger)};
  }

  standardTestSetup(std::string logFilePath) {
    init.horizonSize = nmpcHorizon;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
    std::unique_ptr<VMeModel> mod{new VMeModel{init}};
    std::unique_ptr<NmpcMinimizer> min{new VMeNaiveSdMinimizer{mod.get()}};
    std::unique_ptr<JsonLogger> logger{
        new JsonLogger(mod.get(), logFilePath)};
    eng = new VMeNmpcEngine{std::move(mod), std::move(min),
                                  std::move(logger)};
  }
  ~standardTestSetup() { delete eng; }
  auto* model() { return eng->getModelPointer(); }
  auto* minimizer() { return eng->getMinimizerPointer(); }
};

TEST_CASE(
    "Throw LoggerIsIncompatibleWithModelType if I try to pass an unfamilliar "
    "model to the logger initializer") {
  std::string notUsed;
  std::unique_ptr<vMeModel> mod(new FakeVMeModel(notUsed, 10));

  REQUIRE_THROWS_AS(
      std::unique_ptr<VMeLogger> logger{new JsonLogger(mod.get())};
      , LoggerIsIncompatibleWithModelType);
}

TEST_CASE("Straightforward write to stdout with nothing to assert.") {
  standardTestSetup test;
  test.eng->seed(xyth{0, 0, 0}, fp_point2d{5, 0});
}

TEST_CASE("Logger write to file. TODO: Assert against file contents.") {
  standardTestSetup test("loggertest.log");
  test.eng->seed(xyth{0, 0, 0}, fp_point2d{5, 0});
}