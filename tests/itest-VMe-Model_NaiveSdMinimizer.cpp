#include "catch.hpp"

#include "../src/VMeNmpcEngine.hpp"
#include "../src/NmpcModels/VMeModel.hpp"
#include "../src/NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "../src/Loggers/JsonLogger.hpp"
#include "FakeExecutor.hpp"

#include "../src/trig.hpp"

using std::unique_ptr;
using std::make_unique;

struct TestSetup {
  unsigned int nmpcHorizon = 30;
  float timeInterval = 0.1f;
  float speed = .4;

  AggregatorInitializer init;
  unique_ptr<VMeModel> model{nullptr};
  unique_ptr<VMeNaiveSdMinimizer> minimizer{nullptr};
  unique_ptr<JsonLogger> logger{nullptr};
  unique_ptr<VMeNmpcEngine> engine{nullptr};

void setDefaultParams(InputFileData* parameters) {
    parameters->nmpcHorizon = nmpcHorizon;
    parameters->timeInterval = timeInterval;
    parameters->cruiseSpeed = speed;
    parameters->Q = 1;
    parameters->Q0 = init.parameters->Q / 2;
    parameters->R = init.parameters->Q / 4;
    parameters->sdStepFactor = 0.1;
    parameters->sdConvergenceTolerance = 0.1;
    parameters->maxSdSteps = 1000;
    parameters->targetDistanceTolerance = .1;
}

  TestSetup() {
    setDefaultParams(init.parameters);
    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    logger = make_unique<JsonLogger>(init);
    engine = make_unique<VMeNmpcEngine>(init);
  }

  TestSetup(std::string logFilePath) {
    setDefaultParams(init.parameters);
    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    logger = make_unique<JsonLogger>(init, logFilePath);
    engine = make_unique<VMeNmpcEngine>(init);
  }
};

bool isStopCmd(VMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool isNullCmd(VMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool isMoveCmd(VMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

TEST_CASE("Whatever") {
  TestSetup test{"itest.log.json"};
  FakeExecutor exec(test.engine.get());

  test.engine->seed(xyth{0, 0, 0}, fp_point2d{3, 4});
  while (isMoveCmd(exec.commandFromLastNotify.get())) {
    test.engine->seed(xyth{
        test.model->getX()[1], test.model->getY()[1],
        test.model->getTh()[1],
    });
  }
}