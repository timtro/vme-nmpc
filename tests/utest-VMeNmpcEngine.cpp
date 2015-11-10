#include "catch.hpp"

#include "../src/VMeNmpcEngine.hpp"
#include "FakeVMeModel.hpp"
#include "FakeVMeMinimizer.hpp"
#include "FakeExecutor.hpp"

using std::unique_ptr;
using std::make_unique;

struct TestObject {
  std::string callRecord;
  unsigned horizonSize = 5;

  AggregatorInitializer init;
  unique_ptr<FakeVMeModel> model{nullptr};
  unique_ptr<FakeVMeMinimizer> minimizer{nullptr};
  unique_ptr<VMeNmpcEngine> engine{nullptr};

  TestObject() {
    init.horizonSize = horizonSize;
    model = make_unique<FakeVMeModel>(init, callRecord);
    minimizer = make_unique<FakeVMeMinimizer>(init, callRecord);
    engine = make_unique<VMeNmpcEngine>(init);
  }
};

bool isStopCmd(VMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool isNullCmd(VMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool isMoveCmd(VMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

TEST_CASE(
    "When the robot is on the target the controller should return the command "
    "to stop, and should have halted the model.") {
  TestObject test;
  FakeExecutor exec(test.engine.get());
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.engine->seed(xyth{1, 1, 0}, fp_point2d{1, 1});
  // Should have called (S)eed (D)istanceToTarget:
  REQUIRE(test.callRecord == "SD");
  REQUIRE(isStopCmd(exec.commandFromLastNotify.get()));
  REQUIRE(test.engine->isHalted());
}

TEST_CASE(
    "When the robot and target are sufficiently separated, the engine should "
    "orchestrate the minimization of the cost function over the NMPC horizon "
    "and notify observers of success.") {
  TestObject test;
  FakeExecutor exec(test.engine.get());
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.engine->seed(xyth{0, 0, 0}, fp_point2d{5, 5});
  REQUIRE(test.callRecord == "SDOC");
  REQUIRE(isMoveCmd(exec.commandFromLastNotify.get()));
}

TEST_CASE(
    "If I ask for more commands than are available from the current horizon, "
    "then start returning null commands.") {
  TestObject test;
  FakeExecutor exec(test.engine.get());
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);
  test.engine->seed(xyth{0, 0, 0}, fp_point2d{5, 5});
  unsigned countReturnedMotionCommands = 0;
  auto command = std::move(exec.commandFromLastNotify);
  for (;;) {
    if (isNullCmd(command.get()))
      break;
    else if (isMoveCmd(command.get())) {
      ++countReturnedMotionCommands;
      command = test.engine->nextCommand();
    }
  }
  REQUIRE(countReturnedMotionCommands == test.horizonSize);
}
