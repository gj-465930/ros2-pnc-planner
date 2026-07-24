#include "gtest/gtest.h"
#include "pnc_planner/lattice_planner.hpp"
#include "pnc_planner/reference_line.hpp"

#include <vector>
#include <cmath>

namespace
{

constexpr double kEps = 1e-6;

pnc_planner::ReferenceLine CreateStraightReferenceLine()
{
  pnc_planner::ReferenceLine ref_line;

  const std::vector<double> x = {0.0, 20.0, 40.0};
  const std::vector<double> y = {0.0, 0.0, 0.0};

  EXPECT_TRUE(ref_line.init(x, y));

  return ref_line;
}

pnc_planner::VehicleInfo CreateCruisingEgo()
{
  pnc_planner::VehicleInfo ego;

  ego.pose.x = 0.0;
  ego.pose.y = 0.0;
  ego.pose.yaw = 0.0;
  ego.v = 5.0;
  ego.a = 0.0;
  ego.current_state = pnc_planner::VehicleState::CRUISING;

  return ego;
}

pnc_planner::LatticePlannerConfig CreatePlannerConfig()
{
  pnc_planner::LatticePlannerConfig config;

  config.max_v = 35.0;
  config.min_v = -0.1;
  config.max_acc = 3.0;
  config.min_acc = -5.0;
  config.max_jerk = 4.0;
  config.max_lat_offset = 3.5;
  config.target_speed = 5.0;
  config.planning_time = 3.0;
  config.w_lat = 1.0;
  config.w_lon = 10.0;
  config.w_offset = 2.0;
  config.w_speed = 1.0;

  return config;
}

TEST(LatticePlannerTest, GeneratesTrajectoryOnStraightReferenceLine)
{
  const auto ref_line = CreateStraightReferenceLine();
  const auto ego = CreateCruisingEgo();
  const auto config = CreatePlannerConfig();

  pnc_planner::LatticePlanner planner(config);
  pnc_planner::Trajectory trajectory;

  const bool flag = planner.plan(ego, ref_line, trajectory);
  ASSERT_TRUE(flag);
  ASSERT_FALSE(trajectory.empty());

  EXPECT_NEAR(trajectory.front().x, ego.pose.x, kEps);
  EXPECT_NEAR(trajectory.front().y, ego.pose.y, kEps);

  for (const auto & point : trajectory) {
    EXPECT_GE(point.v, config.min_v - kEps);
    EXPECT_LE(point.v, config.max_v + kEps);
    EXPECT_GE(point.a, config.min_acc - kEps);
    EXPECT_LE(point.a, config.max_acc + kEps);

    double s = 0.0;
    double l = 0.0;
    ASSERT_TRUE(ref_line.getFrenetPoint(point.x, point.y, s, l));
    EXPECT_LE(std::abs(l), config.max_lat_offset + kEps);
  }

  EXPECT_GT(trajectory.back().x, trajectory.front().x);
}

TEST(LatticePlannerTest, ClearsOutputTrajectoryWhenPlanningFails)
{
  const auto ref_line = CreateStraightReferenceLine();
  auto ego = CreateCruisingEgo();
  ego.pose.x = 39.0;

  const auto config = CreatePlannerConfig();

  pnc_planner::LatticePlanner planner(config);
  pnc_planner::Trajectory trajectory;

  pnc_planner::TrajectoryPoint stale_point;
  stale_point.x = 1.0;
  stale_point.y = 2.0;
  trajectory.push_back(stale_point);

  ASSERT_FALSE(trajectory.empty());

  const bool flag = planner.plan(ego, ref_line, trajectory);

  ASSERT_FALSE(flag);
  EXPECT_TRUE(trajectory.empty());
}

}  // namespace