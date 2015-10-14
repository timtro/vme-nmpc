#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../src/VirtualMeNmpcEngine.hpp"
#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/NmpcMinimizers/VirtualMeSDMinimizer.hpp"
#include "../src/VirtualMeCommand.hpp"

bool isStop(VirtualMeCommand* cmd) { return dynamic_cast<VMeStop*>(cmd); }

TEST_CASE("A system with no target should be stationary") {

  unsigned num_of_intervals{50};
  float time_interval{0.1f};
  float speed{0.0f};
  NmpcInitPkg init;

  init.N = num_of_intervals;
  init.T = time_interval;
  init.cruiseSpeed = speed;

  VirtualMeModel mod{init};
  VirtualMeSDMinimizer sdm{};
  VirtualMeNmpcEngine e(mod, sdm);

  e.setTarget(Point2R{0, 0});

  bool last = false;
  auto cmd = e.nextCommand();
  REQUIRE( isStop(cmd.get()) );
}