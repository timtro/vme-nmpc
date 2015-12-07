#include "catch.hpp"

#include "../src/NmpcModels/VMeModel.hpp"
#include "../src/ObstacleTypes/PointObstacle.hpp"
#include "../src/trig.hpp"
#include "FakeVMeMinimizer.hpp"
#include "test_helpers.hpp"  // thlp:: namespace.

using std::unique_ptr;
using std::make_unique;

class TestObject {
 public:
  unsigned nmpcHorizon{50};
  float timeInterval{0.1f};
  float cruiseSpeed{0.4};
  AggregatorInitializer init;
  unique_ptr<VMeModel> model;

  TestObject() {
    init.parameters->nmpcHorizon = nmpcHorizon;
    init.parameters->timeInterval = timeInterval;
    init.parameters->cruiseSpeed = cruiseSpeed;

    model = make_unique<VMeModel>(init);
    SeedPackage seed(init.get_nmpcHorizon());
    model->seed(seed);
  }
};

TEST_CASE("Throw appropriately if horizon size is less than reasonable.") {
  AggregatorInitializer badInit;
  badInit.parameters->nmpcHorizon = 0;
  REQUIRE_THROWS_AS(auto model = make_unique<VMeModel>(badInit),
                    HorizonSizeShouldBeSensiblyLarge);
}

TEST_CASE(
    "Must throw appropriately if the initPkg has been previously used to "
    "initialize a model") {
  AggregatorInitializer badInit;
  badInit.parameters->nmpcHorizon = 3;
  make_unique<VMeModel>(badInit);
  REQUIRE_THROWS_AS(make_unique<VMeModel>(badInit),
                    InitPkgCanOnlyBeUsedOnceToInitializeAModel);
}

TEST_CASE(
    "Throw appropriately if the initPkg has been previously used to initialize "
    "a minimizer (which shouldn't be possible if the minimizer is checking).") {
  AggregatorInitializer badInit;
  badInit.parameters->nmpcHorizon = 3;
  std::string unusedStr;
  make_unique<FakeVMeMinimizer>(badInit, unusedStr);
  REQUIRE_THROWS_AS(make_unique<VMeModel>(badInit),
                    ModelMustBeInitializedBeforeMinimizerOrLogger);
}

TEST_CASE(
    "A machine starting at the origin with no control input and velocity "
    "should remain stationary throughout the forecast horizon.") {
  TestObject m;
  m.model->set_v(0.);
  m.model->computeForecast();
  REQUIRE(thlp::eachInArrayIsApprox(m.model->get_x(), 0.0f, 1e-5f));
  REQUIRE(thlp::eachInArrayIsApprox(m.model->get_y(), 0.0f, 1e-5f));
}

template <typename T>
T linearTravelDistance(T cruiseSpeed, T time_interval, int num_of_intervals) {
  return (num_of_intervals - 1) * cruiseSpeed * time_interval;
}

TEST_CASE(
    "A machine posed at the origin pointing in +x with a constant cruiseSpeed "
    "should drive a straight line along the +x-axis in a forecast horizon.") {
  TestObject m;
  m.model->set_v(1);
  REQUIRE(
      thlp::eachInArrayIsApprox(m.model->get_v(), m.model->get_v()[0], 1e-5f));
  m.model->computeForecast();
  REQUIRE(m.model->get_x()[m.model->get_horizonSize() - 1] ==
          Approx(linearTravelDistance(m.model->get_v()[0], m.timeInterval,
                                      m.nmpcHorizon)));
  REQUIRE(thlp::eachInArrayIsApprox(m.model->get_y(), 0.0f, 1e-5f));
}

TEST_CASE(
    "A machine posed at the origin pointing in +y with a constant cruiseSpeed "
    "should drive a straight line along the +y-axis in a forecast horizon") {
  TestObject m;
  SeedPackage seed(m.nmpcHorizon);
  seed.pose = xyth{0, 0, degToRad(90.f)};
  m.model->seed(seed);
  m.model->computeForecast();
  REQUIRE(m.model->get_y()[m.nmpcHorizon - 1] ==
          Approx(linearTravelDistance(m.cruiseSpeed, m.timeInterval,
                                      m.nmpcHorizon)));
  REQUIRE(thlp::eachInArrayIsApprox(m.model->get_x(), 0.0f, 1e-5f));
}

template <typename T>
auto pathLength(const T &x, const T &y) -> decltype(x[0] + y[0]) {
  if (x.size() != y.size()) throw std::logic_error("Arrays differ in size");
  typedef decltype(x[0] + y[0]) ElementType;
  ElementType len = 0.;
  ElementType dx, dy;
  for (unsigned int k = 1; k < x.size(); ++k) {
    dx = x[k] - x[k - 1];
    dy = y[k] - y[k - 1];
    len += sqrt(dx * dx + dy * dy);
  }
  return len;
}

TEST_CASE(
    "Should be able to compute potential gradient along path without throwing "
    "or faulting") {
  TestObject m;
  m.model->computeForecast();
  ObstacleContainer obs;
  obs.pushObstacleUniquePtr(
      unique_ptr<Obstacle>{new PointObstacle{fp_point2d{10, 10}, 2, .12}});
  obs.pushObstacleUniquePtr(
      unique_ptr<Obstacle>{new PointObstacle{fp_point2d{5, 5}, 2, .12}});
  m.model->computePathPotentialGradient(obs);
}