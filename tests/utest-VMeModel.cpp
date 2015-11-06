#include "catch.hpp"

#include "../src/NmpcModels/VMeModel.hpp"
#include "../src/ObstacleTypes/PointObstacle.hpp"
#include "../src/trig.hpp"
#include "test_helpers.hpp"

class TestObject {
 public:
  unsigned horizonSize{50};
  float timeInterval{0.1f};
  float cruiseSpeed{0.4};
  VMeNmpcInitPkg init;
  VMeModel *model;

  TestObject() {
    init.horizonSize = horizonSize;
    init.timeInterval = timeInterval;
    init.cruiseSpeed = cruiseSpeed;

    new VMeModel{init};
    model = dynamic_cast<VMeModel *>(init.model.get());
    model->seed(xyth{0, 0, 0});
    model->setV(cruiseSpeed);
  }
};

TEST_CASE("Throw appropriately if horizon size is less than reasonable.") {
  VMeNmpcInitPkg badInit;
  badInit.horizonSize = 0;
  REQUIRE_THROWS_AS(auto model = std::make_unique<VMeModel>(badInit),
                    HorizonSizeShouldBeSensiblyLarge);
  REQUIRE(badInit._hasInitializedModel_ == false);
}

TEST_CASE(
    "Must throw appropriately if the initPkg has been previously used to "
    "initialize a model") {
  VMeNmpcInitPkg badInit;
  badInit.horizonSize = 3;
  badInit._hasInitializedModel_ = true;
  REQUIRE_THROWS_AS(auto model = std::make_unique<VMeModel>(badInit),
                    InitPkgCanOnlyBeUsedOnceToInitializeAModel);
}

TEST_CASE(
    "Throw appropriately if the initPkg has been previously used to initialize "
    "a minimizer (which shouldn't be possible if the minimizer is checking).") {
  VMeNmpcInitPkg badInit;
  badInit.horizonSize = 3;
  badInit._hasInitializedMinimizer_ = true;
  REQUIRE_THROWS_AS(std::make_unique<VMeModel>(badInit),
                    ModelMustBeInitializedBeforeMinimizerOrLogger);
  REQUIRE(badInit._hasInitializedModel_ == false);
}

TEST_CASE(
    "Throw appropriately if the initPkg has been previously used to initialize "
    "a logger (which shouldn't be possible if the logger is checking).") {
  VMeNmpcInitPkg badInit;
  badInit.horizonSize = 3;
  badInit._hasInitializedLogger_ = true;
  REQUIRE_THROWS_AS(auto model = std::make_unique<VMeModel>(badInit),
                    ModelMustBeInitializedBeforeMinimizerOrLogger);
  REQUIRE(badInit._hasInitializedModel_ == false);
}

TEST_CASE(
    "A machine starting at the origin with no control input and velocity "
    "should remain stationary throughout the forecast horizon.") {
  TestObject m;
  m.model->setV(0.);
  m.model->computeForecast();
  REQUIRE(eachInArrayIsApprox(m.model->getX(), 0.0f, 1e-5f));
  REQUIRE(eachInArrayIsApprox(m.model->getY(), 0.0f, 1e-5f));
}

template <typename T>
T linearTravelDistance(T cruiseSpeed, T time_interval, int num_of_intervals) {
  return (num_of_intervals - 1) * cruiseSpeed * time_interval;
}

TEST_CASE(
    "A machine posed at the origin pointing in +x with a constant cruiseSpeed "
    "should "
    "drive a straight line along the +x-axis in a forecast horizon.") {
  TestObject m;
  m.model->setV(0.0);
  REQUIRE(eachInArrayIsApprox(m.model->getV(), m.model->getV()[0], 1e-5f));
  m.model->computeForecast();
  REQUIRE(m.model->getX()[m.model->getHorizonSize() - 1] ==
          Approx(linearTravelDistance(m.model->getV()[0], m.timeInterval,
                                      m.horizonSize)));
  REQUIRE(eachInArrayIsApprox(m.model->getY(), 0.0f, 1e-5f));
}

TEST_CASE(
    "A machine posed at the origin pointing in +y with a constant cruiseSpeed "
    "should "
    "drive a straight line along the +y-axis in a forecast horizon") {
  TestObject m;
  m.model->seed(xyth{0, 0, degToRad(90.f)});
  m.model->computeForecast();
  REQUIRE(m.model->getY()[m.horizonSize - 1] ==
          Approx(linearTravelDistance(m.cruiseSpeed, m.timeInterval,
                                      m.horizonSize)));
  REQUIRE(eachInArrayIsApprox(m.model->getX(), 0.0f, 1e-5f));
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
    "Tracking errors when the robot travels along +x and target lies on +y "
    "should form isosceles right triangles") {
  TestObject m;
  m.model->computeForecast();
  fp_point2d tgt{0, m.cruiseSpeed * m.timeInterval * m.horizonSize};
  m.model->seed(xyth{0, 0, 0}, tgt);
  m.model->computeTrackingErrors();
  REQUIRE(arraysAreAbsEqual(m.model->getX(), m.model->getEx(), 1e-6));
  REQUIRE(arraysAreAbsEqual(m.model->getX(), m.model->getEy(), 1e-6));
  REQUIRE(arraysAreAbsEqual(m.model->getEx(), m.model->getEy(), 1e-6));
}

TEST_CASE(
    "Should be able to compute potential gradient along path without throwing "
    "or faulting") {
  TestObject m;
  m.model->computeForecast();
  ObstacleContainer obs;
  obs.pushObstacleUniquePtr(
      std::unique_ptr<Obstacle>{new PointObstacle{fp_point2d{10, 10}, 2, .12}});
  obs.pushObstacleUniquePtr(
      std::unique_ptr<Obstacle>{new PointObstacle{fp_point2d{5, 5}, 2, .12}});
  m.model->computePathPotentialGradient(obs);
}