#include "catch.hpp"

#include "FakeVMeModel.hpp"
#include "../src/NmpcMinimizers/VMeNaiveSdMinimizer.hpp"

// TODO(Tim): Test that throw when iteration limit is reached.
// TODO(Tim): Test that steps are reducing gradient.
// TODO(Tim): Test that plain cases converge.

using std::unique_ptr;
using std::make_unique;

class TestObject {
 public:
  unsigned nmpcHorizon{50};
  float timeInterval{0.1f};
  float cruiseSpeed{0.4};
  float sdStepFactor{.1};
  float convergenceTolerance{0.1};
  unsigned maxSteps{1000};

  AggregatorInitializer init;
  unique_ptr<VMeModel> model{nullptr};
  unique_ptr<VMeNaiveSdMinimizer> minimizer{nullptr};
  std::string callRecord;

  TestObject() {
    init.parameters->nmpcHorizon = nmpcHorizon;
    init.parameters->timeInterval = timeInterval;
    init.parameters->cruiseSpeed = cruiseSpeed;

    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    model->seed(xyth{0, 0, 0});
    model->setV(cruiseSpeed);
  }
};

TEST_CASE(
    "Throw appropriately if the initPkg hasn't already initialized a model "
    "(and therefore doesn't contain a unique_ptr to a model to which we "
    "bind)") {
  AggregatorInitializer badInit;
  REQUIRE_THROWS_AS(make_unique<VMeNaiveSdMinimizer>(badInit),
                    InitPkgDoesNotContainPointerToAModel);
}

TEST_CASE(
    "Throw appropriately if initPkg has already been bound to a minimizer") {
  AggregatorInitializer badInit;
  std::string callRecord;

  badInit.parameters->nmpcHorizon = 3;
  auto tmpModel = make_unique<VMeModel>(badInit);
  auto tmpMinimizer = make_unique<VMeNaiveSdMinimizer>(badInit);
  REQUIRE_THROWS_AS(make_unique<VMeNaiveSdMinimizer>(badInit),
                    InitPkgAlreadyHasBoundMinimizer);
}