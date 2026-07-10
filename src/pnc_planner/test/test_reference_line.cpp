#include <vector>

#include "gtest/gtest.h"
#include "pnc_planner/reference_line.hpp"

namespace {

  constexpr double kEps = 1e-6;

  pnc_planner::ReferenceLine CreateStraightReferenceLine() {
    pnc_planner::ReferenceLine ref_line;

    const std::vector<double> x = {0.0, 10.0, 20.0};
    const std::vector<double> y = {0.0, 0.0, 0.0};

    const bool ok = ref_line.init(x, y);
    EXPECT_TRUE(ok);

    return ref_line;
  }

  TEST(ReferenceLineTest, InitializesAndComputesTotalLength) {
    const auto ref_line = CreateStraightReferenceLine();

    EXPECT_NEAR(ref_line.getTotalLength(), 20.0, kEps);
  }

  TEST(ReferenceLineTest, QueriesMiddleWayPoint) {
    const auto ref_line = CreateStraightReferenceLine();

    const auto way_point = ref_line.getWayPoint(10.0);

    EXPECT_NEAR(way_point.s, 10.0, kEps);
    EXPECT_NEAR(way_point.x, 10.0, kEps);
    EXPECT_NEAR(way_point.y, 0.0, kEps);
    EXPECT_NEAR(way_point.heading, 0.0, kEps);
  }

  TEST(ReferenceLineTest, ClampsOutOfRangeQueries) {
    const auto ref_line = CreateStraightReferenceLine();

    const auto start = ref_line.getWayPoint(-1.0);
    EXPECT_NEAR(start.s, 0.0, kEps);
    EXPECT_NEAR(start.x, 0.0, kEps);
    EXPECT_NEAR(start.y, 0.0, kEps);

    const auto end = ref_line.getWayPoint(100.0);
    EXPECT_NEAR(end.s, 20.0, kEps);
    EXPECT_NEAR(end.x, 20.0, kEps);
    EXPECT_NEAR(end.y, 0.0, kEps);
  }

  TEST(ReferenceLineTest, ConvertsCartesianPointToFrenet) {
    const auto ref_line = CreateStraightReferenceLine();

    double s = 0.0;
    double l = 0.0;

    const bool ok = ref_line.getFrenetPoint(10.0, 1.0, s, l);
    EXPECT_TRUE(ok);
    EXPECT_NEAR(s, 10.0, kEps);
    EXPECT_NEAR(l, 1.0, kEps);
  }

  TEST(ReferenceLineTest, ConvertsFrenetPointToCartesian) {
    const auto ref_line = CreateStraightReferenceLine();

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    const bool ok = ref_line.getCartesianPoint(10.0, 1.0, x, y, yaw);
    EXPECT_TRUE(ok);
    EXPECT_NEAR(x, 10.0, kEps);
    EXPECT_NEAR(y, 1.0, kEps);
    EXPECT_NEAR(yaw, 0.0, kEps);
  }
}  // namespace