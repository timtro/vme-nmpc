#include "catch.hpp"

#include "../src/NmpcModels/VirtualMeModel.hpp"
#include "../src/ObstacleTypes/PointObstacle.hpp"
#include "../src/trig.hpp"
#include "test_helpers.hpp"

class StandardTestModel {
 public:
  unsigned int nmpcHorizon{50};
  float timeInterval{0.1f};
  float speed{0.4};
  NmpcInitPkg init;
  std::unique_ptr<VirtualMeModel> model;

  StandardTestModel() {
    init.N = nmpcHorizon;
    init.T = timeInterval;
    init.cruiseSpeed = speed;

    model = std::unique_ptr<VirtualMeModel>{new VirtualMeModel{init}};
    model->seed(xyth{0, 0, 0});
    model->setV(speed);
  }
};

TEST_CASE(
    "A machine starting at the origin with no control input and velocity "
    "should remain stationary throught the forecast horizon.") {
  StandardTestModel m;
  m.model->setV(0.);
  m.model->computeForecast();
  REQUIRE(eachInArrayIsApprox(m.model->getX(), 0.0f, 1e-5f));
  REQUIRE(eachInArrayIsApprox(m.model->getY(), 0.0f, 1e-5f));
}

template <typename T>
T linearTravelDistance(T speed, T time_interval, int num_of_intervals) {
  return (num_of_intervals - 1) * speed * time_interval;
}

TEST_CASE(
    "A machine posed at the origin pointing in +x with a constant speed should "
    "drive a stright line along the +x-axis in a forecast horizon.") {
  StandardTestModel m;
  m.model->setV(0.0);
  REQUIRE(eachInArrayIsApprox(m.model->getV(), m.model->getV()[0], 1e-5f));
  m.model->computeForecast();
  REQUIRE(m.model->getX()[m.model->getHorizonSize() - 1] ==
          Approx(linearTravelDistance(m.model->getV()[0], m.timeInterval,
                                      m.nmpcHorizon)));
  REQUIRE(eachInArrayIsApprox(m.model->getY(), 0.0f, 1e-5f));
}

TEST_CASE(
    "A machine posed at the origin pointing in +y with a constant speed should "
    "drive a stright line along the +y-axis in a forecast horizon") {
  StandardTestModel m;
  m.model->seed(xyth{0, 0, degToRad(90.f)});
  m.model->computeForecast();
  REQUIRE(m.model->getY()[m.nmpcHorizon - 1] ==
          Approx(linearTravelDistance(m.speed, m.timeInterval, m.nmpcHorizon)));
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
  StandardTestModel m;
  m.model->computeForecast();
  fp_point2d tgt{0, m.speed * m.timeInterval * m.nmpcHorizon};
  m.model->seed(xyth{0, 0, 0}, tgt);
  m.model->computeTrackingErrors();
  REQUIRE(arraysAreAbsEqual(m.model->getX(), m.model->getEx(), 1e-6));
  REQUIRE(arraysAreAbsEqual(m.model->getX(), m.model->getEy(), 1e-6));
  REQUIRE(arraysAreAbsEqual(m.model->getEx(), m.model->getEy(), 1e-6));
}

TEST_CASE(
    "Should be able to compute potential gradient along path without trowing "
    "or faulting") {
  StandardTestModel m;
  m.model->computeForecast();
  ObstacleContainer obs;
  obs.pushObstacleUniquePtr(
      std::unique_ptr<Obstacle>{new PointObstacle{fp_point2d{10, 10}, 2, .12}});
  obs.pushObstacleUniquePtr(
      std::unique_ptr<Obstacle>{new PointObstacle{fp_point2d{5, 5}, 2, .12}});
  m.model->computePathPotentialGradient(obs);
}