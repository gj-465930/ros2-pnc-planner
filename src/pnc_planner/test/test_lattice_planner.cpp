#include "gtest/gtest.h"
#include "pnc_planner/lattice_planner.hpp"
#include "pnc_planner/reference_line.hpp"

#include <cmath>
#include <vector>

constexpr double kEps = 1e-6;

static pnc_planner::ReferenceLine CreateStraightReferenceLine()
{
  pnc_planner::ReferenceLine ref_line;

  const std::vector<double> x = {0.0, 20.0, 40.0};
  const std::vector<double> y = {0.0, 0.0, 0.0};

  EXPECT_TRUE(ref_line.init(x, y));

  return ref_line;
}

static pnc_planner::VehicleInfo CreateCruisingEgo()
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

static pnc_planner::LatticePlannerConfig CreatePlannerConfig()
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
  config.terminal_safety_decel = 3.0;
  config.lateral_transition_distance = 12.0;

  config.w_lat = 1.0;
  config.w_lon = 10.0;
  config.w_offset = 2.0;
  config.w_speed = 1.0;
  config.w_lateral_target_change = 1.0;

  config.lateral_samples = {3.5, 0.0, -3.5};

  return config;
}

static pnc_planner::ReferenceLine CreateLongStraightReferenceLine()
{
  pnc_planner::ReferenceLine ref_line;

  const std::vector<double> x = {0.0, 50.0, 100.0};
  const std::vector<double> y = {0.0, 0.0, 0.0};

  EXPECT_TRUE(ref_line.init(x, y));

  return ref_line;
}

namespace
{
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

  const auto & debug = planner.getLastDebugInfo();
  EXPECT_EQ(debug.lateral_candidate_count, 3U);
  EXPECT_GT(debug.longitudinal_candidate_count, 0U);

  EXPECT_EQ(
    debug.evaluated_pair_count,
    debug.valid_pair_count + debug.kinematic_rejection_count + debug.conversion_rejection_count +
      debug.collision_rejection_count + debug.terminal_safety_rejection_count);

  EXPECT_TRUE(debug.selection_found);
  EXPECT_EQ(debug.planning_failure_reason, pnc_planner::PlanningFailureReason::NONE);

  EXPECT_NEAR(debug.selected_lateral_target, 0.0, kEps);
  EXPECT_GT(debug.selected_duration, 0.0);
  EXPECT_GE(debug.selected_cost, 0.0);

  EXPECT_EQ(
    debug.evaluated_pair_count, debug.lateral_candidate_count * debug.longitudinal_candidate_count);

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

  const auto & debug = planner.getLastDebugInfo();

  EXPECT_EQ(debug.lateral_candidate_count, 3U);
  EXPECT_EQ(debug.longitudinal_candidate_count, 0U);
  EXPECT_EQ(debug.evaluated_pair_count, 0U);
  EXPECT_FALSE(debug.selection_found);
  EXPECT_EQ(
    debug.planning_failure_reason,
    pnc_planner::PlanningFailureReason::LONGITUDINAL_GENERATION_FAILED);
}

TEST(LatticePlannerTest, FarObstacleDoesNotAffectPlanning)
{
  const auto ref_line = CreateStraightReferenceLine();
  const auto ego = CreateCruisingEgo();
  const auto config = CreatePlannerConfig();

  pnc_planner::LatticePlanner planner(config);

  pnc_planner::Obstacle obstacle;
  obstacle.x = 100.0;
  obstacle.y = 0.0;
  obstacle.length = 2.0;
  obstacle.width = 1.0;
  obstacle.heading = 0.0;

  planner.setObstacles({obstacle});

  pnc_planner::Trajectory trajectory;
  const bool success = planner.plan(ego, ref_line, trajectory);
  ASSERT_TRUE(success);
  ASSERT_FALSE(trajectory.empty());

  const auto & debug = planner.getLastDebugInfo();
  EXPECT_TRUE(debug.selection_found);
  EXPECT_EQ(debug.collision_rejection_count, 0U);
  EXPECT_EQ(debug.planning_failure_reason, pnc_planner::PlanningFailureReason::NONE);
}

TEST(LatticePlannerTest, BlockingObstacleCausesPlanningFailureAndClearsOutput)
{
  const auto ref_line = CreateStraightReferenceLine();
  const auto ego = CreateCruisingEgo();
  const auto config = CreatePlannerConfig();

  pnc_planner::LatticePlanner planner(config);

  pnc_planner::Obstacle obstacle;
  obstacle.x = 5.0;
  obstacle.y = 0.0;
  obstacle.length = 8.0;
  obstacle.width = 4.0;
  obstacle.heading = 0.0;

  planner.setObstacles({obstacle});

  pnc_planner::Trajectory trajectory;

  pnc_planner::TrajectoryPoint stale_point;
  stale_point.x = 100.0;
  stale_point.y = 100.0;
  trajectory.push_back(stale_point);

  ASSERT_FALSE(trajectory.empty());

  const bool planning_success = planner.plan(ego, ref_line, trajectory);

  EXPECT_FALSE(planning_success);
  EXPECT_TRUE(trajectory.empty());

  const auto & debug = planner.getLastDebugInfo();
  EXPECT_FALSE(debug.selection_found);
  EXPECT_EQ(debug.planning_failure_reason, pnc_planner::PlanningFailureReason::NO_VALID_TRAJECTORY);

  EXPECT_GT(debug.collision_rejection_count, 0U);

  EXPECT_EQ(
    debug.evaluated_pair_count,
    debug.valid_pair_count + debug.kinematic_rejection_count + debug.conversion_rejection_count +
      debug.collision_rejection_count + debug.terminal_safety_rejection_count);
}

TEST(LatticePlannerTest, SelectsSafeCandidateAroundObstacle)
{
  const auto ref_line = CreateStraightReferenceLine();
  const auto ego = CreateCruisingEgo();

  auto config = CreatePlannerConfig();
  config.planning_time = 5.0;

  pnc_planner::LatticePlanner planner(config);

  pnc_planner::Obstacle obstacle;
  obstacle.x = 15.0;
  obstacle.y = 0.0;
  obstacle.length = 1.0;
  obstacle.width = 1.0;
  obstacle.heading = 0.0;

  planner.setObstacles({obstacle});

  pnc_planner::Trajectory trajectory;
  const bool planning_success = planner.plan(ego, ref_line, trajectory);

  ASSERT_TRUE(planning_success);
  ASSERT_FALSE(trajectory.empty());
  const double safe_dist = (3.0 + obstacle.length) / 2.0;

  bool has_lateral_offset = false;

  for (const auto & point : trajectory) {
    const double dx = point.x - obstacle.x;
    const double dy = point.y - obstacle.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    EXPECT_GE(distance, safe_dist - kEps);

    if (std::abs(point.y) > 0.5) {
      has_lateral_offset = true;
    }
  }

  EXPECT_TRUE(has_lateral_offset);
}

TEST(LatticePlannerTest, ReportsLateAvoidanceFailureAtCriticalPosition)
{
  const auto ref_line = CreateStraightReferenceLine();
  auto ego = CreateCruisingEgo();

  auto config = CreatePlannerConfig();
  config.planning_time = 5.0;

  pnc_planner::LatticePlanner planner(config);

  pnc_planner::Obstacle obstacle;
  obstacle.x = 20.0;
  obstacle.y = 0.0;
  obstacle.length = 1.0;
  obstacle.width = 1.0;
  obstacle.heading = 0.0;

  planner.setObstacles({obstacle});

  pnc_planner::Trajectory initial_trajectory;

  ASSERT_TRUE(planner.plan(ego, ref_line, initial_trajectory));
  ASSERT_FALSE(initial_trajectory.empty());

  const auto initial_debug = planner.getLastDebugInfo();

  EXPECT_TRUE(initial_debug.selection_found);
  EXPECT_NEAR(std::abs(initial_debug.selected_lateral_target), 3.5, kEps);
  EXPECT_NEAR(initial_debug.selected_duration, config.planning_time, kEps);
  EXPECT_GT(initial_debug.collision_rejection_count, 0U);

  ego.pose.x = 14.0;

  pnc_planner::Trajectory critical_trajectory;

  ASSERT_FALSE(planner.plan(ego, ref_line, critical_trajectory));
  EXPECT_TRUE(critical_trajectory.empty());

  const auto & critical_debug = planner.getLastDebugInfo();

  EXPECT_FALSE(critical_debug.selection_found);
  EXPECT_EQ(critical_debug.valid_pair_count, 0U);
  EXPECT_GT(critical_debug.collision_rejection_count, 0U);
  EXPECT_EQ(
    critical_debug.planning_failure_reason,
    pnc_planner::PlanningFailureReason::NO_VALID_TRAJECTORY);

  EXPECT_EQ(
    critical_debug.evaluated_pair_count,
    critical_debug.valid_pair_count + critical_debug.kinematic_rejection_count +
      critical_debug.conversion_rejection_count + critical_debug.collision_rejection_count +
      critical_debug.terminal_safety_rejection_count);
}

TEST(LatticePlannerTest, RejectsUnsafeTerminalStateAndSelectsAvoidance)
{
  const auto ref_line = CreateStraightReferenceLine();
  const auto ego = CreateCruisingEgo();

  auto config = CreatePlannerConfig();
  config.planning_time = 5.0;

  pnc_planner::LatticePlanner planner(config);

  pnc_planner::Obstacle obstacle;
  obstacle.x = 23.0;
  obstacle.y = 0.0;
  obstacle.length = 1.0;
  obstacle.width = 1.0;
  obstacle.heading = 0.0;

  planner.setObstacles({obstacle});

  pnc_planner::Trajectory trajectory;

  ASSERT_TRUE(planner.plan(ego, ref_line, trajectory));
  ASSERT_FALSE(trajectory.empty());

  const auto & debug = planner.getLastDebugInfo();

  EXPECT_TRUE(debug.selection_found);
  EXPECT_NEAR(debug.selected_duration, config.planning_time, kEps);
  EXPECT_GT(debug.collision_rejection_count, 0U);

  EXPECT_GT(debug.terminal_safety_rejection_count, 0U);

  EXPECT_NEAR(std::abs(debug.selected_lateral_target), 3.5, kEps);
}

TEST(LatticePlannerTest, HoldsLateralTargetAfterLateralProfileEnds)
{
  const auto ref_line = CreateStraightReferenceLine();
  const auto ego = CreateCruisingEgo();

  auto config = CreatePlannerConfig();
  config.planning_time = 5.0;
  config.target_speed = 7.0;

  config.w_lat = 0.0;
  config.w_lon = 0.0;
  config.w_offset = 0.0;
  config.w_speed = 1.0;

  pnc_planner::LatticePlanner planner(config);

  pnc_planner::Obstacle obstacle;
  obstacle.x = 15.0;
  obstacle.y = 0.0;
  obstacle.length = 1.0;
  obstacle.width = 1.0;
  obstacle.heading = 0.0;

  planner.setObstacles({obstacle});

  pnc_planner::Trajectory trajectory;

  ASSERT_TRUE(planner.plan(ego, ref_line, trajectory));
  ASSERT_FALSE(trajectory.empty());

  const auto & debug = planner.getLastDebugInfo();

  EXPECT_NEAR(std::abs(debug.selected_lateral_target), 3.5, kEps);

  EXPECT_GT(trajectory.back().x, 25.0);
  EXPECT_NEAR(trajectory.back().v, 7.0, 1e-3);
  EXPECT_NEAR(std::abs(trajectory.back().y), 3.5, 1e-3);
  EXPECT_NEAR(trajectory.back().heading, 0.0, 1e-3);
  EXPECT_NEAR(trajectory.back().kappa, 0.0, 1e-3);
}

TEST(LatticePlannerTest, ReplansContinuouslyAroundStaticObstacle)
{
  const auto ref_line = CreateLongStraightReferenceLine();
  auto ego = CreateCruisingEgo();

  auto config = CreatePlannerConfig();
  config.planning_time = 5.0;
  config.w_lateral_target_change = 100.0;
  double avoidance_direction = 0.0;

  pnc_planner::LatticePlanner planner(config);

  pnc_planner::Obstacle obstacle;
  obstacle.x = 20.0;
  obstacle.y = 0.0;
  obstacle.length = 1.0;
  obstacle.width = 1.0;
  obstacle.heading = 0.0;

  planner.setObstacles({obstacle});

  constexpr std::size_t max_replan_count = 100;
  bool passed_obstacle = false;
  bool formed_lateral_offset = false;

  for (std::size_t iteration = 0; iteration < max_replan_count; ++iteration) {
    pnc_planner::Trajectory trajectory;

    const bool planning_success = planner.plan(ego, ref_line, trajectory);

    const auto debug = planner.getLastDebugInfo();

    ASSERT_TRUE(planning_success) << "iteration: " << iteration << ", ego_x: " << ego.pose.x
                                  << ", ego_y: " << ego.pose.y << ", ego_v: " << ego.v
                                  << ", evaluated: " << debug.evaluated_pair_count
                                  << ", valid: " << debug.valid_pair_count
                                  << ", kinematic: " << debug.kinematic_rejection_count
                                  << ", conversion: " << debug.conversion_rejection_count
                                  << ", collision: " << debug.collision_rejection_count
                                  << ", terminal_safety: " << debug.terminal_safety_rejection_count;

    if (const auto & debug_info = planner.getLastDebugInfo();
        std::abs(debug_info.selected_lateral_target) > kEps) {
      const double current_direction = std::copysign(1.0, debug_info.selected_lateral_target);

      if (avoidance_direction == 0.0) {
        avoidance_direction = current_direction;
      } else {
        EXPECT_EQ(current_direction, avoidance_direction) << "iteration: " << iteration;
      }
    }

    ASSERT_GT(trajectory.size(), 1U) << "iteration: " << iteration;

    const double safe_dist = (3.0 + obstacle.length) / 2.0;

    for (const auto & point : trajectory) {
      const double dx = point.x - obstacle.x;
      const double dy = point.y - obstacle.y;
      const double distance = std::sqrt(dx * dx + dy * dy);

      EXPECT_GE(distance, safe_dist - kEps) << "iteration: " << iteration;
    }

    const auto & next_state = trajectory[1];

    ego.pose.x = next_state.x;
    ego.pose.y = next_state.y;
    ego.pose.yaw = next_state.heading;
    ego.v = next_state.v;
    ego.a = next_state.a;

    if (std::abs(ego.pose.y) > 0.5) {
      formed_lateral_offset = true;
    }

    if (ego.pose.x > obstacle.x + safe_dist) {
      passed_obstacle = true;
      break;
    }
  }
  EXPECT_TRUE(passed_obstacle);
  EXPECT_TRUE(formed_lateral_offset);
}

TEST(LatticePlannerTest, UsesConfiguredLateralSamples)
{
  const auto ref_line = CreateStraightReferenceLine();
  const auto ego = CreateCruisingEgo();

  auto config = CreatePlannerConfig();
  config.lateral_samples = {2.0, 1.0, 0.0, -1.0, -2.0};

  pnc_planner::LatticePlanner planner(config);
  pnc_planner::Trajectory trajectory;

  ASSERT_TRUE(planner.plan(ego, ref_line, trajectory));

  const auto & debug = planner.getLastDebugInfo();
  EXPECT_EQ(debug.lateral_candidate_count, 5U);
}

}  // namespace
