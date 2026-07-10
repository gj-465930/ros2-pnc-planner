#include "gtest/gtest.h"
#include "pnc_planner/math/cartesian_frenet.hpp"

namespace {

  constexpr double kEps = 1e-6;
  constexpr double kMaxS = 50.0;

  void EvaluateStraightReferenceLine(double &s, double &x, double &y, double &heading) {
    x = s;
    y = 0.0;
    heading = 0.0;
  }

  TEST(CartesianFrenetTest, LeftPointHasPositiveL) {
    double s = 0.0;
    double l = 0.0;

    const bool ok = pnc_planner::math::CartesianFrenetConverter::cartesianToFrenet(
      10.0, 2.0, kMaxS, EvaluateStraightReferenceLine, s, l);

    EXPECT_TRUE(ok);
    EXPECT_NEAR(s, 10.0, kEps);
    EXPECT_NEAR(l, 2.0, kEps);
  }

  TEST(CartesianFrenetTest, RightPointHasPositiveL) {
    double s = 0.0;
    double l = 0.0;

    const bool ok = pnc_planner::math::CartesianFrenetConverter::frenetToCartesian(
      10.0, -2.0, kMaxS, EvaluateStraightReferenceLine, s, l);

    EXPECT_TRUE(ok);
    EXPECT_NEAR(s, 10.0, kEps);
    EXPECT_NEAR(l, -2.0, kEps);
  }

  TEST(CartesianFrenetTest, RoundTripReturnsNearOriginalPoint) {
    double s = 0.0;
    double l = 0.0;

    ASSERT_TRUE(pnc_planner::math::CartesianFrenetConverter::cartesianToFrenet(
      10.0, 2.0, kMaxS, EvaluateStraightReferenceLine, s, l));

    double x = 0.0;
    double y = 0.0;
    
    ASSERT_TRUE(pnc_planner::math::CartesianFrenetConverter::frenetToCartesian(
      s, l, kMaxS, EvaluateStraightReferenceLine, x, y));

    EXPECT_NEAR(x, 10.0, kEps);
    EXPECT_NEAR(y, 2.0, kEps);
  }
} // namespace
