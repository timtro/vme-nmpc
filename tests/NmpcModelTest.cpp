#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/NmpcModel.hpp"
#include "test_helpers.hpp"

NmpcModel stationaryTestModel() {
  int num_of_intervals = 50;
  float time_interval = 0.1f;
  float speed = 0;

  NmpcInitPkg init_a;
  init_a.N = num_of_intervals;
  init_a.T = time_interval;
  init_a.cruising_speed = speed;
  return NmpcModel(init_a);
}

NmpcModel standardTestModel() {
  float speed = 0.1f;
  auto m = stationaryTestModel();
  m.v = speed;
  m.cruising_speed = speed;
  m.seed();
  return m;
}

TEST_CASE("A machine starting at the origin with no control input and no"
          " velocity should be stationary") {
  auto m = stationaryTestModel();
  m.forecast();
  REQUIRE(eachInArrayIsApprox(m.x, 0.0f, 1e-3f));
  REQUIRE(eachInArrayIsApprox(m.y, 0.0f, 1e-3f));
}

template <typename T>
T linearTravelDistance(T speed, T time_interval, int num_of_intervals) {
  return (num_of_intervals-1) * speed * time_interval;
}

TEST_CASE("A machine posed at the origin pointing in +x with a constant speed"
          " should drive a stright line along the +x-axis") {
  auto m = standardTestModel();
  REQUIRE(eachInArrayIsApprox(m.v, m.v[0], 1e-3f));
  m.forecast();
  REQUIRE(m.x[m.N-1] == Approx(
            linearTravelDistance(m.v[0], m.T, m.N)
          ));
  REQUIRE(eachInArrayIsApprox(m.y, 0.0f, 1e-3f));
}

TEST_CASE("A machine posed at the origin pointing in +y with a constant speed"
                  " should drive a stright line along the +y-axis") {
  auto m = standardTestModel();
  m.seed(XYVTh<float>{0, 0, m.cruising_speed, 90.f});
  m.forecast();
  REQUIRE(m.y[m.N-1] == Approx(
          linearTravelDistance(m.v[0], m.T, m.N)
  ));
  REQUIRE(eachInArrayIsApprox(m.x, 0.0f, 1e-5f));
}

template <typename T>
auto pathLength(T x, T y) -> decltype(x[0]+y[0]) {
  if (x.size() != y.size()) throw std::logic_error("Arrays differ in size");
  typedef decltype(x[0] + y[0]) ElementType;
  ElementType len = 0.;
  ElementType dx, dy;
  for (unsigned int k = 1; k < x.size(); ++k) {
    dx = x[k] - x[k-1];
    dy = y[k] - y[k-1];
    len += sqrt(dx*dx + dy*dy);
  }
  return len;
}

TEST_CASE("Path length should not depend on steering rate") {
  auto m = standardTestModel();
  m.Dth = 1;
  m.seed();
  m.forecast();
  REQUIRE(pathLength(m.x, m.y) == Approx(linearTravelDistance(m.v[0], m.T, m.N)));
}