#include "catch.hpp"

#include "../src/VirtualMeNmpcEngine.hpp"
#include "FakeVirtualMeModel.hpp"
#include "FakeMinimizer.hpp"
#include "FakeExecutor.hpp"

const int standardTestHorizon = 10;
struct standardTestSetup {
  VirtualMeNmpcEngine* eng{nullptr};

  standardTestSetup() {
    std::unique_ptr<vMeModel> mod{
        new FakeVirtualMeModel{standardTestHorizon}};
    std::unique_ptr<NmpcMinimizer> min{new FakeMinimizer{}};
    eng = new VirtualMeNmpcEngine{std::move(mod), std::move(min)};
  }
  ~standardTestSetup() {
    delete eng;
  }
  auto* model() {
    return dynamic_cast<FakeVirtualMeModel*>(eng->getModelPointer());
  }
  auto* minimizer() {
    return dynamic_cast<FakeMinimizer*>(eng->getMinimizerPointer());
  }
};

bool isStopCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool isNullCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool isMoveCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

TEST_CASE("Throw appropriately if we try to attach the same observer twice.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  // NB: the FakeExecutor's constructor does the first attach.
  REQUIRE_THROWS_AS(test.eng->attachObserver(&exec),
                    AttemptToAttachAlreadyAttachedObserver);
}

TEST_CASE(
    "When the robot is on the target the controller should return the command "
    "to stop, and should have halted the model.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.eng->seed(xyvth{1, 1, 0, 0}, fp_point2d{1, 1});
  // Should have called (S)eed (D)istanceToTarget and (H)alt:
  REQUIRE(test.model()->getEventHistory() == "SD");
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
  test.eng->seed(xyvth{0, 0, 0, 0}, fp_point2d{5, 5});
  REQUIRE(test.model()->getEventHistory() == "SDC");
  REQUIRE(test.minimizer()->getEventHistory() == "O");
  REQUIRE(isMoveCmd(exec.commandFromLastNotify.get()));
}

TEST_CASE(
    "If I ask for more commands than are available from the current horizon, "
    "then start returning null commands.") {
  standardTestSetup test;
  FakeExecutor exec(test.eng);
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.eng->seed(xyvth{0, 0, 0, 0}, fp_point2d{5, 5});
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
  REQUIRE(countReturnedMotionCommands == standardTestHorizon);
}