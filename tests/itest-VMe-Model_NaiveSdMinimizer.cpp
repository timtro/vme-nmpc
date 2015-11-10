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
  unsigned int nmpcHorizon = 50;
  float timeInterval = 0.1f;
  float speed = .4;

  VMeNmpcInitPkg init;
  unique_ptr<VMeModel> model{nullptr};
  unique_ptr<VMeNaiveSdMinimizer> minimizer{nullptr};
  unique_ptr<JsonLogger> logger{nullptr};
  unique_ptr<VMeNmpcEngine> engine{nullptr};

  TestSetup() {
    init.horizonSize = nmpcHorizon;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
    init.R = init.Q / 4;
    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    logger = make_unique<JsonLogger>(init);
    engine = make_unique<VMeNmpcEngine>(init);
  }

  TestSetup(std::string logFilePath) {
    init.horizonSize = nmpcHorizon;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
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