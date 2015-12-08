#include "catch.hpp"

#include "../src/VMeNmpcKernel.hpp"
#include "FakeVMeModel.hpp"
#include "FakeVMeMinimizer.hpp"
#include "FakeExecutor.hpp"
#include "FakePathPlanner.hpp"

using std::unique_ptr;
using std::make_unique;

struct TestObject {
  std::string callRecord;
  unsigned nmpcHorizon = 5;

  AggregatorInitializer init;
  unique_ptr<FakeVMeModel> model{nullptr};
  unique_ptr<FakeVMeMinimizer> minimizer{nullptr};
  unique_ptr<PathPlanner<SeedPackage>> planner{nullptr};
  unique_ptr<VMeNmpcKernel> engine{nullptr};

  TestObject() {
    init.parameters->nmpcHorizon = nmpcHorizon;
    model = make_unique<FakeVMeModel>(init, callRecord);
    minimizer = make_unique<FakeVMeMinimizer>(init, callRecord);
    planner = make_unique<FakePathPlanner>(init);
    engine = make_unique<VMeNmpcKernel>(init);
  }
};

bool isStopCmd(VMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool isNullCmd(VMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool isMoveCmd(VMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

TEST_CASE("Throw appropriately if AggregatorInitializer is missing a model") {
  std::string callRecord;
  AggregatorInitializer init;
  // Fake minimizer let's us cheat and initialize without model.
  auto minimizer = make_unique<FakeVMeMinimizer>(init, callRecord);
  REQUIRE_THROWS_AS(make_unique<VMeNmpcKernel>(init),
                    InitPkgDoesNotContainPointerToAModel);
}

TEST_CASE(
    "Throw appropriately if AggregatorInitializer is missing a minimizer") {
  std::string callRecord;
  AggregatorInitializer init;
  auto model = make_unique<FakeVMeModel>(init, callRecord);
  REQUIRE_THROWS_AS(make_unique<VMeNmpcKernel>(init),
                    InitPkgDoesNotContainPointerToAMinimizer);
}

TEST_CASE(
    "When the robot is on the target the controller should return the command "
    "to stop, and should have halted the model.") {
  TestObject test;
  FakeExecutor exec(test.engine.get());
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);

  SeedPackage seed(test.nmpcHorizon);
  seed.pose = xyth{1, 1, 0};
  test.engine->seed(seed);
  // Should have called (S)eed (D)istanceToTarget:
  REQUIRE(test.callRecord == "SD");
  REQUIRE(isStopCmd(exec.commandFromLastNotify.get()));
}

TEST_CASE(
    "When the robot and target are sufficiently separated, the engine should "
    "orchestrate the minimization of the cost function over the NMPC horizon "
    "and notify observers of success.") {
  TestObject test;
  FakeExecutor exec(test.engine.get());
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);

  SeedPackage seed(test.nmpcHorizon);
  seed.pose = xyth{0, 0, 0};
  test.engine->seed(seed);
  REQUIRE(test.callRecord == "SDOC");
  REQUIRE(isMoveCmd(exec.commandFromLastNotify.get()));
}

TEST_CASE(
    "If I ask for more commands than are available from the current horizon, "
    "then start returning null commands.") {
  TestObject test;
  FakeExecutor exec(test.engine.get());
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);

  SECTION("Newly created engine should provide null commands:") {
    auto command = test.engine->nextCommand();
    REQUIRE(isNullCmd(command.get()));
  }

  SECTION(
      "Run an NMPC step and count to be sure that no more than a horizon's "
      "worth of commands are doled out.") {
    SeedPackage seed(test.nmpcHorizon);
    test.engine->nmpcStep(seed);
    unsigned countReturnedMotionCommands = 0;
    auto command = std::move(exec.commandFromLastNotify);
    for (;;) {
      if (isNullCmd(command.get()))
        break;
      else if (isMoveCmd(command.get())) {
        ++countReturnedMotionCommands;
        command = test.engine->nextCommand();
      } else
        throw std::runtime_error(
            "ERROR: Unexpected output from engine.nextCommand().\n");
    }
    REQUIRE(countReturnedMotionCommands == test.nmpcHorizon);
  }
}
