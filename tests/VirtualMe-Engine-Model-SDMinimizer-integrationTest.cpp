#include "catch.hpp"

#include "../src/VirtualMeNmpcEngine.hpp"
#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/NmpcMinimizers/VirtualMeSDMinimizer.hpp"

struct standardTestSetup {
  VirtualMeModel* mod{nullptr};
  VirtualMeSDMinimizer* min{nullptr};
  VirtualMeNmpcEngine* eng{nullptr};
  unsigned int nmpcHorizon = 50;
  float timeInterval = 0.1f;
  float speed = .4;
  NmpcInitPkg init;

  standardTestSetup() {
    init.N = nmpcHorizon;
    init.T = timeInterval;
    init.cruiseSpeed = speed;
    mod = new VirtualMeModel{init};
    min = new VirtualMeSDMinimizer{mod};
    eng = new VirtualMeNmpcEngine{*mod, *min};
  }
  ~standardTestSetup() {
    delete eng;
    delete min;
    delete mod;
  }
};
