#include "pnc_planner/controller/pure_pursuit_controller.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace pnc_planner::controller
{
TrajectoryPoint PurePursuitController::findLookaheadPoint(
  const Trajectory & traj, const VehicleInfo & ego)
{
  double ld = std::max(ld_min_, ego.v * ld_ratio_);
  double cos_yaw = std::cos(ego.pose.yaw);
  double sin_yaw = std::sin(ego.pose.yaw);

  size_t start = 0;
  double min_dist = std::numeric_limits<double>::max();

  // 寻找距离自车最近的轨迹点
  for (size_t i = 0; i < traj.size(); ++i) {
    double dx = traj[i].x - ego.pose.x;
    double dy = traj[i].y - ego.pose.y;
    double dist = dx * dx + dy * dy;

    if (dist < min_dist) {
      min_dist = dist;
      start = i;
    }
  }

  // 确保起点是在自车前方
  for (size_t i = start; i < traj.size(); ++i) {
    double dx = traj[i].x - ego.pose.x;
    double dy = traj[i].y - ego.pose.y;

    double proj = dx * cos_yaw + dy * sin_yaw;
    if (proj >= 0.0) {
      start = i;
      break;
    }
  }

  // 寻找第一个 >= ld的点
  for (size_t i = start; i < traj.size(); ++i) {
    double dx = traj[i].x - ego.pose.x;
    double dy = traj[i].y - ego.pose.y;

    if (dx * dx + dy * dy >= ld * ld) {
      return traj[i];
    }
  }

  return traj.back();
}

double PurePursuitController::computeYawRate(const Trajectory & traj, const VehicleInfo & ego)
{
  if (traj.empty()) return 0.0;

  TrajectoryPoint target = findLookaheadPoint(traj, ego);

  double dx = target.x - ego.pose.x;
  double dy = target.y - ego.pose.y;
  double yaw = ego.pose.yaw;

  double local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
  double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);

  double actual_ld_sq = local_x * local_x + local_y * local_y;

  if (actual_ld_sq < 0.001) {
    return 0.0;
  }

  double delta = std::atan2(2.0 * wheelbase_ * local_y, actual_ld_sq);

  // 安全防护
  constexpr double max_steer = 0.61;  // 约 35 度
  delta = std::clamp(delta, -max_steer, max_steer);

  double omega = ego.v / wheelbase_ * std::tan(delta);
  return omega;
}
}  // namespace pnc_planner::controller