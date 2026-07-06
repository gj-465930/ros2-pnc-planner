#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "pnc_planner/math/quintic_polynomial.hpp"

namespace {
  constexpr double kEps = 1e-6;

  TEST(QuinticPolynomialTest, SatisfiesBoundaryConditions) {
    const double x0 = 0.0;
    const double v0 = 0.0;
    const double a0 = 0.0;
    const double x1 = 10.0;
    const double v1 = 0.0;
    const double a1 = 0.0;
    const double T = 5.0;

    pnc_planner::math::QuinticPolynomial poly(x0, v0, a0, x1, v1, a1, T);

    EXPECT_NEAR(poly.evaluate(0.0), x0, kEps);
    EXPECT_NEAR(poly.evaluate_d(0.0), v0, kEps);
    EXPECT_NEAR(poly.evaluate_dd(0.0), a0, kEps);

    EXPECT_NEAR(poly.evaluate(T), x1, kEps);
    EXPECT_NEAR(poly.evaluate_d(T), v1, kEps);
    EXPECT_NEAR(poly.evaluate_dd(T), a1, kEps);
  }

  TEST(QuinticPolynomialTest, SampledValuesAreFinite) {
    pnc_planner::math::QuinticPolynomial poly(
      0.0, 0.0, 0.0,
      10.0, 0.0, 0.0,
      5.0
    );

    const std::vector<double> sample_times = {0.0, 1.0, 2.5, 5.0};

    for (const double t : sample_times) {
      EXPECT_TRUE(std::isfinite(poly.evaluate(t)));
      EXPECT_TRUE(std::isfinite(poly.evaluate_d(t)));
      EXPECT_TRUE(std::isfinite(poly.evaluate_dd(t)));
      EXPECT_TRUE(std::isfinite(poly.evaluate_ddd(t)));
    }
  }
} // namespace