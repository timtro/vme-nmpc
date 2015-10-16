#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../src/VirtualMeNmpcEngine.hpp"
#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/NmpcMinimizers/VirtualMeSDMinimizer.hpp"
#include "../src/VirtualMeCommand.hpp"

struct standardTestSetup {
  NmpcModel* mod{nullptr};
  NmpcMinimizer* min{nullptr};
  VirtualMeNmpcEngine* eng{nullptr};

  standardTestSetup() {
    unsigned num_of_intervals{5};
    NmpcInitPkg init;
    init.N = num_of_intervals;

    mod = new VirtualMeModel{init};
    min = new VirtualMeSDMinimizer{};
    eng = new VirtualMeNmpcEngine{*mod, *min};
  }
  ~standardTestSetup() {
    delete eng;
    delete min;
    delete mod;
  }
};

bool isStopCmd(VirtualMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }

TEST_CASE(
    "When the robot is on the target the controller shold return"
    " the command to stop.") {
  standardTestSetup test;
  test.eng->setTarget(Point2R{0, 0});
  auto cmd = test.eng->nextCommand();
  REQUIRE(isStopCmd(cmd.get()));
}

TEST_CASE(
    "If I ask for more commands than are available from the current"
    " calculation, then start returning stop commanda.") {}

TEST_CASE(
    "Given a non-originated target, the controller should try to"
    " iterate on the min mock, and get convergence in iterations.") {}