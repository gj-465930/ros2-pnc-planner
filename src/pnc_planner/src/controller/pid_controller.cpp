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

  const std::size_t target_index = std::min<std::size_t>(1, traj.size() - 1);

  const double feedforward_accel = traj[target_index].a;

  const double error = traj.front().v - ego.v;

  // P
  const double p_term = kp_ * error;

  // I
  integral_ += error * dt_;
  integral_ = std::clamp(integral_, -max_integral_, max_integral_);
  const double i_term = ki_ * integral_;

  // D
  const double derivative = (error - previous_error_) / dt_;
  previous_error_ = error;
  const double d_term = kd_ * derivative;

  const double feedback_accel = p_term + i_term + d_term;

  // 轨迹加速度前馈 + 速度误差反馈
  const double accel = feedforward_accel + feedback_accel;

  return std::clamp(accel, min_acc_, max_acc_);
}
}  // namespace pnc_planner::controller