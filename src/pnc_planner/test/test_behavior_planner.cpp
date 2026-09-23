#include "gtest/gtest.h"
#include "pnc_planner/planning/behavior/behavior_planner.hpp"

#include <vector>

static pnc_planner::ReferenceLine CreateStraightReferenceLine()
{
  pnc_planner::ReferenceLine reference_line;

  const std::vector<double> x = {0.0, 10.0, 20.0, 30.0, 40.0};
  const std::vector<double> y = {0.0, 0.0, 0.0, 0.0, 0.0};

  EXPECT_TRUE(reference_line.init(x, y));
  return reference_line;
}

static pnc_planner::planning::behavior::BehaviorPlannerConfig CreateBehaviorPlannerConfig()
{
  pnc_planner::planning::behavior::BehaviorPlannerConfig config;

  config.cruise_speed = 8.0;
  config.route_end_stop_buffer = 2.0;
  config.comfortable_decel = 3.0;
  config.stop_trigger_margin = 1.0;

  return config;
}

namespace
{

TEST(BehaviorPlannerTest, CruisesWhenFarFromRouteEnd)
{
  const auto reference_line = CreateStraightReferenceLine();
  const auto planner_config = CreateBehaviorPlannerConfig();

  pnc_planner::planning::behavior::BehaviorPlanner planner(planner_config);

  pnc_planner::VehicleInfo ego;
  ego.pose.x = 5.0;
  ego.pose.y = 0.0;
  ego.v = 2.0;

  const auto result = planner.plan(ego, reference_line);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->behavior, pnc_planner::planning::BehaviorState::CRUISE);
  EXPECT_DOUBLE_EQ(result->target_speed, planner_config.cruise_speed);
  EXPECT_FALSE(result->stop_s.has_value());
}

TEST(BehaviorPlannerTest, StopsWhenBrakingShouldStart)
{
  const auto reference_line = CreateStraightReferenceLine();
  const auto planner_config = CreateBehaviorPlannerConfig();

  pnc_planner::planning::behavior::BehaviorPlanner planner(planner_config);

  pnc_planner::VehicleInfo ego;
  ego.pose.x = 31.0;
  ego.pose.y = 0.0;
  ego.v = 6.0;

  const auto result = planner.plan(ego, reference_line);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->behavior, pnc_planner::planning::BehaviorState::STOP);
  EXPECT_DOUBLE_EQ(result->target_speed, 0.0);

  ASSERT_TRUE(result->stop_s.has_value());
  EXPECT_DOUBLE_EQ(*result->stop_s, 38.0);
}

TEST(BehaviorPlannerTest, KeepsStopWhenStopIsTriggered)
{
  const auto reference_line = CreateStraightReferenceLine();
  const auto planner_config = CreateBehaviorPlannerConfig();

  pnc_planner::planning::behavior::BehaviorPlanner planner(planner_config);

  pnc_planner::VehicleInfo ego;
  ego.pose.x = 31.0;
  ego.pose.y = 0.0;
  ego.v = 6.0;

  const auto first_result = planner.plan(ego, reference_line);
  ASSERT_TRUE(first_result.has_value());
  EXPECT_EQ(first_result->behavior, pnc_planner::planning::BehaviorState::STOP);

  ego.v = 0.0;

  const auto second_result = planner.plan(ego, reference_line);
  ASSERT_TRUE(second_result.has_value());
  EXPECT_EQ(second_result->behavior, pnc_planner::planning::BehaviorState::STOP);
}

TEST(BehaviorPlannerTest, RejectsInvalidConfiguration)
{
  const auto reference_line = CreateStraightReferenceLine();
  auto planner_config = CreateBehaviorPlannerConfig();

  planner_config.comfortable_decel = 0.0;

  pnc_planner::planning::behavior::BehaviorPlanner planner(planner_config);

  pnc_planner::VehicleInfo ego;
  ego.pose.x = 5.0;
  ego.pose.y = 0.0;
  ego.v = 2.0;

  const auto result = planner.plan(ego, reference_line);
  ASSERT_FALSE(result.has_value());
}

TEST(BehaviorPlannerTest, RejectsUninitializedReferenceLine)
{
  const auto config = CreateBehaviorPlannerConfig();
  const pnc_planner::ReferenceLine reference_line;

  pnc_planner::planning::behavior::BehaviorPlanner planner(config);

  pnc_planner::VehicleInfo ego;
  ego.pose.x = 0.0;
  ego.pose.y = 0.0;
  ego.v = 2.0;

  const auto result = planner.plan(ego, reference_line);
  ASSERT_FALSE(result.has_value());
}

}  // namespace
