#pragma once

#include <cstdint>

#include "geometry_msgs/msg/point.hpp"

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
  double a = 0.0;
  double omega = 0.0; // 横摆角速度

  VehicleState current_state = VehicleState::INIT;
};

struct WayPoint {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0; // 航向角
  double kappa = 0.0;   // 曲率
  double s = 0.0;       // 弧长
};

struct TrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;

  double v = 0.0;
  double a = 0.0;
  double kappa = 0.0;

  double t = 0.0;
};

using Trajectory = std::vector<TrajectoryPoint>;

} // namespace pnc_planner