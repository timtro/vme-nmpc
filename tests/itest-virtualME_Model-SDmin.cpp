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
  auto* model() { return eng->getModelPointer(); }
  auto* minimizer() { return eng->getMinimizerPointer(); }
};

TEST_CASE("Whatever") {
  standardTestSetup test{"Blah"};
  FakeExecutor exec(test.eng);
  test.eng->seed(xyvth{0, 0, test.speed, 0}, fp_point2d{5, 0});
}