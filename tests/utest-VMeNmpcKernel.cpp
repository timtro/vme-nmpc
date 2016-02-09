#include "catch.hpp"

#include "../src/VMeNmpcKernel.hpp"
#include "FakeExecutor.hpp"
#include "FakePathPlanner.hpp"
#include "FakeVMeMinimizer.hpp"
#include "FakeVMeModel.hpp"

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

bool is_stop_cmd(VMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }
bool is_null_cmd(VMeCommand* cmd) { return dynamic_cast<VMeNullCmd*>(cmd); }
bool is_motion_cmd(VMeCommand* cmd) { return dynamic_cast<VMeV*>(cmd); }

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
    "If I ask for more commands than are available from the current horizon, "
    "then start returning null commands.") {
  TestObject test;
  FakeExecutor exec(test.engine.get());
  REQUIRE(exec.commandFromLastNotify.get() == nullptr);

  SECTION("Newly created engine should provide null commands:") {
    auto command = test.engine->next_command();
    REQUIRE(is_null_cmd(command.get()));
  }

  SECTION(
      "Run an NMPC step and count to be sure that no more than a horizon's "
      "worth of commands are doled out.") {
    SeedPackage seed(test.nmpcHorizon);
    test.engine->nmpc_step(seed);
    unsigned countReturnedMotionCommands = 0;
    auto command = std::move(exec.commandFromLastNotify);
    for (;;) {
      if (is_null_cmd(command.get()))
        break;
      else if (is_motion_cmd(command.get())) {
        ++countReturnedMotionCommands;
        command = test.engine->next_command();
      } else
        throw std::runtime_error(
            "ERROR: Unexpected output from engine.next_command().\n");
    }
    REQUIRE(countReturnedMotionCommands == test.nmpcHorizon);
  }
}
