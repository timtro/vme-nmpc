#include "catch.hpp"

#include "FakeVMeModel.hpp"
#include "../src/NmpcMinimizers/VMeNaiveSdMinimizer.hpp"

// TODO(Tim): Test that throw when iteration limit is reached.
// TODO(Tim): Test that steps are reducing gradient.
// TODO(Tim): Test that plain cases converge.

class TestObject {
 public:
  unsigned horizonSize{50};
  float timeInterval{0.1f};
  float cruiseSpeed{0.4};
  float sdStepFactor{.1};
  float convergenceTolerance{0.1};
  unsigned maxSteps{1000};
  VMeNmpcInitPkg init;
  VMeModel *model;
  VMeNaiveSdMinimizer *minimizer;
  std::string callRecord;

  TestObject() {
    init.horizonSize = horizonSize;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = cruiseSpeed;

    new FakeVMeModel{init, callRecord};
    new VMeNaiveSdMinimizer{init};
    model = dynamic_cast<VMeModel *>(init.model.get());
    model->seed(xyth{0, 0, 0});
    model->setV(cruiseSpeed);
    minimizer = dynamic_cast<VMeNaiveSdMinimizer *>(init.minimizer.get());
  }
};

TEST_CASE(
    "Throw appropriately if the initPkg hasn't already initialized a model "
    "(and therefore doesn't contain a unique_ptr to a model to which we "
    "bind)") {
  VMeNmpcInitPkg badInit;
  REQUIRE_THROWS_AS(new VMeNaiveSdMinimizer(badInit),
                    InitPkgDoesNotContainPointerToAModel);
}

TEST_CASE(
    "Throw appropriately if initPkg has already been bound to a minimizer") {
  VMeNmpcInitPkg badInit;
  std::string callRecord;
  new FakeVMeModel{badInit, callRecord};
  new VMeNaiveSdMinimizer(badInit);
  REQUIRE_THROWS_AS(new VMeNaiveSdMinimizer(badInit),
                    InitPkgAlreadyHasBoundMinimizer);
}