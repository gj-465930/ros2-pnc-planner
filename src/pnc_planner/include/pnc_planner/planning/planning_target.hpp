#pragma once

#include <cstdint>
#include <optional>

namespace pnc_planner::planning
{

enum class BehaviorState : std::uint8_t { CRUISE = 0, STOP };

struct PlanningTarget
{
  BehaviorState behavior = BehaviorState::CRUISE;
  double target_speed = 0.0;
  std::optional<double> stop_s;
};

} // namespace pnc_planner::planning
