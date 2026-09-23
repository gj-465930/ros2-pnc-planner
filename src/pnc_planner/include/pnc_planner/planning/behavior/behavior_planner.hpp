#pragma once

#include "pnc_planner/common.hpp"
#include "pnc_planner/planning/planning_target.hpp"
#include "pnc_planner/reference_line.hpp"

#include <optional>

namespace pnc_planner::planning::behavior
{

struct BehaviorPlannerConfig
{
  double cruise_speed = 0.0;
  double route_end_stop_buffer = 0.0;
  double comfortable_decel = 0.0;
  double stop_trigger_margin = 0.0;
};

class BehaviorPlanner
{
public:
  explicit BehaviorPlanner(const BehaviorPlannerConfig & config);

  std::optional<PlanningTarget> plan(
    const pnc_planner::VehicleInfo & ego,
    const pnc_planner::ReferenceLine & reference_line);

  void reset();

private:
  bool isConfigValid() const;

  BehaviorPlannerConfig config_;
  bool stop_latched_ = false;
};

}  // namespace pnc_planner::planning::behavior
