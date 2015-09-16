#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "../src/NmpcModel.hpp"
#include <iostream>



TEST_CASE("A machine starting at the origin with no control input and no"
          " expected velocity should be stationary") {
  NmpcInitPkg init_a{20, 1, 5, .12, .1, .4, 10, 10, 5};
  NmpcModel m(init_a);
  m.v = 0.0f;
  m.forecast();
  REQUIRE(m.x.sum() == 0.f);
  REQUIRE(m.y.sum() == 0.f);
}

float linearTravelDistance(float speed, float time_interval,
                           int num_of_intervals) {
  return (num_of_intervals-1) * speed * time_interval;
}

TEST_CASE("A machine starting at the origin with a constant speed should drive"
          " a stright line along the +x-axis") {
  int num_of_intervals = 10;
  float speed = 1;
  float time_interval = 1;
  NmpcInitPkg init_a;

  init_a.N = num_of_intervals;
  init_a.T = time_interval;
  init_a.cruising_speed = speed;
  NmpcModel m(init_a);
  m.v = m.cruising_speed;
  m.Dx[0] = m.v[0];
  m.forecast();
  REQUIRE(m.x[num_of_intervals - 1] == Approx( linearTravelDistance(speed,
          time_interval,
          num_of_intervals) ));
  REQUIRE(m.y.sum() == 0);
}