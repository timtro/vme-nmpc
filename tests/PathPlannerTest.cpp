#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include <iostream>

TEST_CASE("A newly minted NmpcEngine should have an empty stack of targets") {
  NmpcInitPkg init_a{20, 1, 5, .12, .1, .4, 10, 10, 5};
  NmpcEngine e(init_a);

  REQUIRE(e.numberOfTargets() == 0);
  e.pushFinalTarget(Target{5., 5., .1});

  SECTION(
      "and adding a target to the back and popping should leave the stack"
      " empty") {
    REQUIRE(e.numberOfTargets() == 1);
    e.popFinalTarget();
    REQUIRE(e.numberOfTargets() == 0);
  }

  SECTION(
      "now adding a target to front and back should give me the expected"
      " order") {
    Target a{1, 2, 3};
    Target b{4, 5, 6};
    e.pushFinalTarget(b);
    e.pushCurrentTarget(a);
    REQUIRE(e.currentTarget().x == 1.);
    REQUIRE(e.currentTarget().locus.y == 2.);
    REQUIRE(e.finalTarget().locus.x == 4.);
    REQUIRE(e.finalTarget().y == 5.);
  }

  SECTION("Target's x,y members should be references to the fp_point2d locus") {
    Target a{1, 2, 3};
    a.x = 99.f;
    REQUIRE(a.locus.x == 99.f);
  }

  SECTION("Clearing the stack should leave it empty") {
    e.clearTargetList();
    REQUIRE(e.areTargetsRemaining() == false);
  }
}