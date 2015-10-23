#include "catch.hpp"

#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/ObstacleTypes/PointObstacle.hpp"
#include "../src/trig.hpp"
#include "test_helpers.hpp"

auto stationaryTestModel() {
  unsigned int num_of_intervals = 50;
  float time_interval = 0.1f;
  float speed = 0;

  NmpcInitPkg init;
  init.N = num_of_intervals;
  init.T = time_interval;
  init.cruiseSpeed = speed;
  return VirtualMeModel(init);
}

auto standardTestModel() {
  float speed = 0.1f;
  auto m = stationaryTestModel();
  m.v = speed;
  m.cruiseSpeed = speed;
  m.seed(xyvth{0, 0, m.cruiseSpeed, 0});
  return m;
}

TEST_CASE(
    "A machine starting at the origin with no control input and no"
    " velocity should be stationary") {
  auto m = stationaryTestModel();
  m.computeForecast();
  REQUIRE(eachInArrayIsApprox(m.x, 0.0f, 1e-5f));
  REQUIRE(eachInArrayIsApprox(m.y, 0.0f, 1e-5f));
}

template <typename T>
T linearTravelDistance(T speed, T time_interval, int num_of_intervals) {
  return (num_of_intervals - 1) * speed * time_interval;
}

TEST_CASE(
    "A machine posed at the origin pointing in +x with a constant speed"
    " should drive a stright line along the +x-axis") {
  auto m = standardTestModel();
  REQUIRE(eachInArrayIsApprox(m.v, m.v[0], 1e-5f));
  m.computeForecast();
  REQUIRE(m.x[m.N - 1] == Approx(linearTravelDistance(m.v[0], m.T, m.N)));
  REQUIRE(eachInArrayIsApprox(m.y, 0.0f, 1e-5f));
}

TEST_CASE(
    "A machine posed at the origin pointing in +y with a constant speed"
    " should drive a stright line along the +y-axis") {
  auto m = standardTestModel();
  m.th[0] = degToRad(90);
  m.seed(xyvth{0, 0, m.cruiseSpeed, 90});
  m.computeForecast();
  REQUIRE(m.y[m.N - 1] == Approx(linearTravelDistance(m.v[0], m.T, m.N)));
  REQUIRE(eachInArrayIsApprox(m.x, 0.0f, 1e-5f));
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

TEST_CASE("Path length should not depend on steering rate") {
  auto m = standardTestModel();
  m.Dth = 1;
  m.seed(xyvth{0, 0, m.cruiseSpeed, 45});
  m.computeForecast();
  REQUIRE(pathLength(m.x, m.y) ==
          Approx(linearTravelDistance(m.v[0], m.T, m.N)));
}

TEST_CASE(
    "Tracking errors when the robot is along +x and target is on +y"
    " should form isosceles right triangles") {
  auto m = standardTestModel();
  m.computeForecast();
  fp_point2d tgt{0, m.x[m.N - 1]};
  m.seed(xyvth{0, 0, m.cruiseSpeed, 0}, tgt);
  m.computeTrackingErrors();
  REQUIRE(arraysAreAbsEqual(m.x, m.ex, 1e-6));
  REQUIRE(arraysAreAbsEqual(m.x, m.ey, 1e-6));
  REQUIRE(arraysAreAbsEqual(m.ex, m.ey, 1e-6));
}

TEST_CASE(
    "Should be able to compute potential gradient along path without"
    " trowing or faulting") {
  auto m = standardTestModel();
  m.computeForecast();
  ObstacleContainer obs;
  obs.pushObstacleUniquePtr(
      std::unique_ptr<Obstacle>{new PointObstacle{fp_point2d{10, 10}, 2, .12}});
  obs.pushObstacleUniquePtr(
      std::unique_ptr<Obstacle>{new PointObstacle{fp_point2d{5, 5}, 2, .12}});
  m.computePathPotentialGradient(obs);
}

TEST_CASE(
    "Should be able to compute Lagrange multipliers without"
    " trowing or faulting") {
  auto m = standardTestModel();
  m.computeForecast();
  ObstacleContainer obs;
  obs.pushObstacle(new PointObstacle{fp_point2d{10, 10}, 2, .12});
  obs.pushObstacle(new PointObstacle{fp_point2d{5, 5}, 2, .12});
  m.computePathPotentialGradient(obs);
  m.computeLagrageMultipliers();
}