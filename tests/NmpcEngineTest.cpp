#define CATCH_CONFIG_MAIN

#include "catch.hpp"

#include "../src/NmpcEngine.hpp"
#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/NmpcMinimizers/VirtualMeSDMinimizer.hpp"

auto standardTestEngine() {
  unsigned num_of_intervals{50};
  float time_interval{0.1f};
  float speed{.4f};

  NmpcInitPkg init;
  init.N = num_of_intervals;
  init.T = time_interval;
  init.cruiseSpeed = speed;
  auto e = std::unique_ptr<NmpcModel>{std::move(new VirtualMeModel(init))};
  auto m = std::unique_ptr<NmpcMinimizer>{std::move(new VirtualMeSDMinimizer())};
  return NmpcEngine(init, std::move(e), std::move(m));
}

TEST_CASE("A system with no target should be stationary") {
  auto e = standardTestEngine();
}