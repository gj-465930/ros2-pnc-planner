#pragma once

#include <cstdint>

namespace pnc_planner {

enum class VehicleState : uint8_t {
  INIT = 0,
  STANDBY = 1,
  CRUISING = 2,
  EMERGENCY = 3
};

struct Pose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct VehicleInfo {
  Pose pose;
  double v = 0.0;
  double omega = 0.0; // 横摆角速度

  VehicleState current_state = VehicleState::INIT;
};

} // namespace pnc_planner