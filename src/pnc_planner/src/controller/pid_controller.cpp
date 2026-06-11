#include "pnc_planner/controller/pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pnc_planner::controller
{

double PidController::computeAccel(const Trajectory & traj, const VehicleInfo & ego)
{
  if (traj.empty()) {
    return -2.0;
  }

  // 寻找自车附近的轨迹点
  double cos_yaw = std::cos(ego.pose.yaw);
  double sin_yaw = std::sin(ego.pose.yaw);

  size_t start = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < traj.size(); ++i) {
    double dx = traj[i].x - ego.pose.x;
    double dy = traj[i].y - ego.pose.y;
    double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      start = i;
    }
  }

  // 确保轨迹点在车辆的前方
  for (size_t i = start; i < traj.size(); ++i) {
    double dx = traj[i].x - ego.pose.x;
    double dy = traj[i].y - ego.pose.y;
    double proj = dx * cos_yaw + dy * sin_yaw;
    if (proj >= 0.0) {
      start = i;
      break;
    }
  }

  double target_v = traj[start].v;
  double error = target_v - ego.v;

  // pid控制
  // p
  double p_term = kp_ * error;

  // i
  integral_ += error * dt_;
  integral_ = std::clamp(integral_, -max_integral_, max_integral_);
  double i_term = ki_ * integral_;

  // d
  double derivative = (error - previous_error_) / dt_;
  previous_error_ = error;
  double d_term = kd_ * derivative;

  double accel = p_term + i_term + d_term;
  return std::clamp(accel, min_acc_, max_acc_);
}
}  // namespace pnc_planner::controller