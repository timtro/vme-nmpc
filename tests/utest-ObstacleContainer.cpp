#include "catch.hpp"

#include "../src/Obstacle.hpp"
#include "../src/ObstacleTypes/PointObstacle.hpp"

std::random_device rd;
std::default_random_engine e1(rd());

std::uniform_real_distribution<float> zero_to_ten(1., 10.);
std::uniform_real_distribution<float> zero_to_one(0., 1.);

TEST_CASE("Distances are computed correctly") {
  SECTION("For pointObstacles, we expect a Pythagorean relationship") {
    REQUIRE(PointObstacle(fp_point2d{0, 0}, 0, 0).dist(fp_point2d{1, 1}) ==
            Approx(std::sqrt(2)));
    // Some pythagorean tripplets
    REQUIRE(PointObstacle(fp_point2d{0, 1}, 0, 0).dist(fp_point2d{1, 1}) ==
            Approx(1));
    REQUIRE(PointObstacle(fp_point2d{-20, 0}, 0, 0).dist(fp_point2d{0, -99}) ==
            Approx(101));
    REQUIRE(PointObstacle(fp_point2d{0, 68}, 0, 0).dist(fp_point2d{285, 0}) ==
            Approx(293));
  }
}

TEST_CASE("Potential values are computed correctly") {
  SECTION("For pointObstacles, fall off like 1/r^pwr") {
    /*
     * Test that
     *                    p
     *   phi(x + 1)      x  + eps
     *   ---------- = --------------- .
     *     phi(x)            pwr
     *                (x + 1)  + eps
     *
     */

    // since the maximum exponent size we'll test for is 10, do 10th root of max
    // float _less the max offset) to be the biggest number we can get here.
    std::uniform_real_distribution<float> bigr(
        0, pow(std::numeric_limits<float>::max() - 10, 0.1));

    for (int k = 0; k < 10; ++k) {
      float pwr = zero_to_ten(e1), eps = zero_to_one(e1);
      auto uv = unitVector(30.f);
      float x = bigr(e1);
      float offset = zero_to_one(e1);
      auto a = uv * x;
      auto b = uv * (x + offset);
      auto tc = PointObstacle(fp_point2d{0, 0}, pwr, eps);

      REQUIRE(
          tc.phi(b) / tc.phi(a) ==
          Approx((std::pow(x, pwr) + eps) / (std::pow(x + offset, pwr) + eps)));
    }
  }
}

TEST_CASE("Gradients are computed correctly... a though one.") {
  /*
   * The best way I can think of testing this is just to use some hard numbers
   * computed from wxMaxima
   *
   * Phi(x,y) := 1/(sqrt(x^2 + y^2)^pwr + eps); [diff(Phi(x,y), x),
   * diff(Phi(x,y), y)]; G(x,y) :=
   * [-(pwr*x*(y^2+x^2)^(pwr/2-1))/((y^2+x^2)^(pwr/2)+eps)^2,-(pwr*y*(y^2+x^2)^(pwr/2-1))/((y^2+x^2)^(pwr/2)+eps)^2];
   */
  // float(ev(G(3, 4), pwr=2, eps=.2));
  // > [-0.00944822373393802,-0.01259763164525069]
  auto tc = PointObstacle(fp_point2d{0, 0}, 2, .2);
  REQUIRE(tc.gradient_phi(fp_point2d(3, 4)).x == Approx(-0.00944822373393802));
  REQUIRE(tc.gradient_phi(fp_point2d(3, 4)).y == Approx(-0.01259763164525069));
  // float(ev(G(5, -2), pwr=4, eps=.12));
  // > [-8.198078531413808*10^-4,3.279231412565523*10^-4]
  tc = PointObstacle(fp_point2d{0, 0}, 4, .12);
  REQUIRE(tc.gradient_phi(fp_point2d(5, -2)).x ==
          Approx(-8.198078531413808E-4));
  REQUIRE(tc.gradient_phi(fp_point2d(5, -2)).y == Approx(3.279231412565523E-4));
}

TEST_CASE("Create a stack of obstacles, fill and empty it") {
  ObstacleContainer obs;
  REQUIRE(obs.has_obstacles() == false);
  REQUIRE(obs.size() == 0);
  // The long way:
  obs.push_unique_ptr(
      std::unique_ptr<Obstacle>{new PointObstacle{fp_point2d{10, 10}, 2, .12}});
  REQUIRE(obs.size() == 1);
  REQUIRE(obs.has_obstacles() == true);
  // The short(er) way:
  obs.push(new PointObstacle{fp_point2d{5, 5}, 2, .12});
  REQUIRE(obs.size() == 2);
  obs.pop();
  REQUIRE(obs.size() == 1);
  obs.clear();
  REQUIRE(obs.has_obstacles() == false);
}

TEST_CASE(
    "Given two point obstacles at a, the gradient at a should be"
    " zero (i.e., we are on the peak).") {
  ObstacleContainer obs;
  obs.push(new PointObstacle{fp_point2d{1, 1}, 2, 2});
  obs.push(new PointObstacle{fp_point2d{1, 1}, 2, 2});
  fp_point2d a{1, 1};
  fp_point2d grad = obs.gradient_phi(a);
  REQUIRE(grad.x == Approx(0));
  REQUIRE(grad.y == Approx(0));
}
