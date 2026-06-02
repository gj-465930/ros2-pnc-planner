#pragma once

#include <cstdint>

#include "geometry_msgs/msg/point.hpp"

namespace pnc_planner {

// 车辆状态机
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

struct LatticePlannerConfig {
  double max_v = 0.0;
  double min_v = 0.0;
  double max_acc = 0.0;
  double min_acc = 0.0;
  double max_jerk = 0.0;
  double max_lat_offset = 0.0;
};

struct Obstacle {
  double x = 0.0, y = 0.0;
  double length = 0.0, width = 0.0;
  double heading = 0.0;
};

} // namespace pnc_planner