#include "catch.hpp"

#include "../src/VirtualMeNmpcEngine.hpp"
#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/NmpcMinimizers/VirtualMeSDMinimizer.hpp"
#include "../src/Loggers/StdoutJsonLogger.hpp"
#include "FakeExecutor.hpp"

struct standardTestSetup {
  VirtualMeNmpcEngine* eng{nullptr};
  unsigned int nmpcHorizon = 50;
  float timeInterval = 0.1f;
  float speed = .4;
  NmpcInitPkg init;

  standardTestSetup() {
    init.N = nmpcHorizon;
    init.T = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
    std::unique_ptr<VirtualMeModel> mod{new VirtualMeModel{init}};
    std::unique_ptr<VirtualMeSDMinimizer> min{
        new VirtualMeSDMinimizer{mod.get()}};
    std::unique_ptr<StdoutJsonLogger> logger{new StdoutJsonLogger(mod.get())};
    eng = new VirtualMeNmpcEngine{std::move(mod), std::move(min),
                                  std::move(logger)};
  }

  standardTestSetup(std::string logFilePath) {
    init.N = nmpcHorizon;
    init.T = timeInterval;
    init.cruiseSpeed = speed;
    init.Q = 1;
    init.Q0 = init.Q / 2;
    std::unique_ptr<VirtualMeModel> mod{new VirtualMeModel{init}};
    std::unique_ptr<NmpcMinimizer> min{new VirtualMeSDMinimizer{mod.get()}};
    std::unique_ptr<StdoutJsonLogger> logger{
        new StdoutJsonLogger(mod.get(), logFilePath)};
    eng = new VirtualMeNmpcEngine{std::move(mod), std::move(min),
                                  std::move(logger)};
  }
  ~standardTestSetup() { delete eng; }
  auto* model() {
    return dynamic_cast<VirtualMeModel*>(eng->getModelPointer());
  }
  auto* minimizer() { return eng->getMinimizerPointer(); }
};

bool isStopCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool isNullCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool isMoveCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

TEST_CASE("Whatever") {
  standardTestSetup test{"itest.log.json"};
  FakeExecutor exec(test.eng);

  test.eng->seed(xyvth{0, 0, test.speed, 0}, fp_point2d{5, 0});
  while (isMoveCmd(exec.commandFromLastNotify.get())) {
    test.eng->seed(xyvth{
        test.model()->getX()[1], test.model()->getY()[1],
        test.model()->getV()[1], test.model()->getTh()[1],
    });
  }
}