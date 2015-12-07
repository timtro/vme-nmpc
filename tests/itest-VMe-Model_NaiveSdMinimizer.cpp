#include "catch.hpp"

#include "../src/VMeNmpcKernel.hpp"
#include "../src/NmpcModels/VMeModel.hpp"
#include "../src/PathPlanners/VMePathPlanner.hpp"
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
  unique_ptr<TargetContainer> targets{nullptr};
  unique_ptr<ObstacleContainer> obstacles{nullptr};
  unique_ptr<VMeModel> model{nullptr};
  unique_ptr<VMeNaiveSdMinimizer> minimizer{nullptr};
  unique_ptr<VMePathPlanner> planner{nullptr};
  unique_ptr<JsonLogger> logger{nullptr};
  unique_ptr<VMeNmpcKernel> engine{nullptr};

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
    targets = make_unique<TargetContainer>();
    obstacles = make_unique<ObstacleContainer>();
    init.targets = targets.get();
    init.obstacles = obstacles.get();
    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    logger = make_unique<JsonLogger>(init);
    planner = make_unique<VMePathPlanner>(init);
    engine = make_unique<VMeNmpcKernel>(init);
  }

  TestSetup(std::string logFilePath) {
    setDefaultParams(init.parameters);
    targets = make_unique<TargetContainer>();
    obstacles = make_unique<ObstacleContainer>();
    init.targets = targets.get();
    init.obstacles = obstacles.get();
    model = make_unique<VMeModel>(init);
    minimizer = make_unique<VMeNaiveSdMinimizer>(init);
    logger = make_unique<JsonLogger>(init, logFilePath);
    planner = make_unique<VMePathPlanner>(init);
    engine = make_unique<VMeNmpcKernel>(init);
  }
};

bool isStopCmd(VMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool isNullCmd(VMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool isMoveCmd(VMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

TEST_CASE("Whatever") {
  TestSetup test{"itest.log.json"};
  FakeExecutor exec(test.engine.get());

  test.planner->set_stateEstimateRetriever([&test](){
    SeedPackage pose;
    pose.pose.x = test.model->get_x()[1];
    pose.pose.y = test.model->get_y()[1];
    pose.pose.th = test.model->get_th()[1];
    return pose;
  });

  test.targets->push_back(make_unique<Target>(5, 10, 0.1));
  test.engine->run();
}