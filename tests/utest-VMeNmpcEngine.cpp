#include "catch.hpp"

#include "../src/VMeNmpcEngine.hpp"
#include "FakeVMeModel.hpp"
#include "FakeVMeMinimizer.hpp"
#include "FakeExecutor.hpp"

struct standardTestSetup {
  VMeNmpcEngine* eng{nullptr};
  std::string callRecord;
  VMeNmpcInitPkg init;
  unsigned horizonSize = 5;

  standardTestSetup() {
    init.horizonSize = horizonSize;
    new FakeVMeModel{init, callRecord};
    new FakeVMeMinimizer{init, callRecord};
    eng = new VMeNmpcEngine{init};
  }
  ~standardTestSetup() {
    delete eng;
  }
  auto* model() {
    return dynamic_cast<FakeVMeModel*>(eng->_getModelPointer_());
  }
  auto* minimizer() {
    return dynamic_cast<FakeVMeMinimizer*>(eng->_getMinimizerPointer_());
  }
};

bool isStopCmd(VMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool isNullCmd(VMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool isMoveCmd(VMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }



TEST_CASE(
    "When the robot is on the target the controller should return the command "
    "to stop, and should have halted the model.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.eng->seed(xyth{1, 1, 0}, fp_point2d{1, 1});
  // Should have called (S)eed (D)istanceToTarget and (H)alt:
  REQUIRE(test.callRecord == "SD");
  REQUIRE(isStopCmd(exec.commandFromLastNotify.get()));
  REQUIRE(test.eng->isHalted());
}

TEST_CASE(
    "When the robot and target are sufficiently separated, the engine should "
    "orchestrate the minimization of the cost function over the NMPC horizon "
    "and notify observers of success.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.eng->seed(xyth{0, 0, 0}, fp_point2d{5, 5});
  REQUIRE(test.callRecord == "SDOC");
  REQUIRE(isMoveCmd(exec.commandFromLastNotify.get()));
}

TEST_CASE(
    "If I ask for more commands than are available from the current horizon, "
    "then start returning null commands.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.eng->seed(xyth{0, 0, 0}, fp_point2d{5, 5});
  unsigned countReturnedMotionCommands = 0;
  auto command = std::move(exec.commandFromLastNotify);
  for (;;) {
    if (isNullCmd(command.get()))
      break;
    else if (isMoveCmd(command.get())) {
      ++countReturnedMotionCommands;
      command = test.eng->nextCommand();
    }
  }
  REQUIRE(countReturnedMotionCommands == test.horizonSize);
}