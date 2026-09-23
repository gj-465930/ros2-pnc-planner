#include "pnc_planner/planning/behavior/behavior_planner.hpp"

#include <cmath>

namespace pnc_planner::planning::behavior
{
BehaviorPlanner::BehaviorPlanner(const BehaviorPlannerConfig & config) : config_(config)
{
}

bool BehaviorPlanner::isConfigValid() const
{
  return std::isfinite(config_.cruise_speed) && config_.cruise_speed >= 0.0 &&
         std::isfinite(config_.route_end_stop_buffer) && config_.route_end_stop_buffer >= 0.0 &&
         std::isfinite(config_.comfortable_decel) && config_.comfortable_decel > 0.0 &&
         std::isfinite(config_.stop_trigger_margin) && config_.stop_trigger_margin >= 0.0;
}

std::optional<PlanningTarget> BehaviorPlanner::plan(
  const pnc_planner::VehicleInfo & ego, const pnc_planner::ReferenceLine & reference_line)
{
  if (!isConfigValid()) {
    return std::nullopt;
  }

  const double reference_length = reference_line.getTotalLength();

  if (!std::isfinite(reference_length) || reference_length <= 0.0) {
    return std::nullopt;
  }

  const double stop_s = reference_length - config_.route_end_stop_buffer;

  if (!std::isfinite(stop_s) || stop_s <= 0.0) {
    return std::nullopt;
  }

  double ego_s = 0.0;
  double ego_l = 0.0;

  if (!reference_line.getFrenetPoint(ego.pose.x, ego.pose.y, ego_s, ego_l)) {
    return std::nullopt;
  }

  if (
    !std::isfinite(ego_s) || !std::isfinite(ego_l) || !std::isfinite(ego.v) || ego_s < 0.0 ||
    ego.v < 0.0) {
    return std::nullopt;
  }

  if (stop_latched_) {
    PlanningTarget target;
    target.behavior = BehaviorState::STOP;
    target.target_speed = 0.0;
    target.stop_s = stop_s;
    return target;
  }

  const double braking_distance = ego.v * ego.v / (2 * config_.comfortable_decel);

  const double remaining_distance = stop_s - ego_s;

  if (remaining_distance <= braking_distance + config_.stop_trigger_margin) {
    stop_latched_ = true;

    PlanningTarget target;
    target.behavior = BehaviorState::STOP;
    target.target_speed = 0.0;
    target.stop_s = stop_s;
    return target;
  }

  PlanningTarget target;
  target.behavior = BehaviorState::CRUISE;
  target.target_speed = config_.cruise_speed;
  target.stop_s = std::nullopt;
  return target;
}

void BehaviorPlanner::reset()
{
  stop_latched_ = false;
}

}  // namespace pnc_planner::planning::behavior