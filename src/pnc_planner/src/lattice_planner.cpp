#include "pnc_planner/lattice_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace pnc_planner {

bool LatticePlanner::plan(const VehicleInfo &ego, const ReferenceLine &ref_line,
                          Trajectory &out_trajectory) {
  // 生成横向候选轨迹
  auto lat_trajs = generate_lateral_trajectories(ego, ref_line);
  if (lat_trajs.empty()) {
    std::cerr << "[LatticePlanner] Error: 横向轨迹生成失败" << std::endl;
    return false;
  }

  // 生成纵向候选轨迹
  auto lon_trajs = generate_longitudinal_trajectories(ego);
  if (lon_trajs.empty()) {
    std::cerr << "[LatticePlanner] Error: 纵向轨迹生成失败" << std::endl;
    return false;
  }

  // cost evaluation
  auto best_indices = evaluate_and_select_best_trajectory(lat_trajs, lon_trajs);
  int best_lat_idx = best_indices.first;
  int best_lon_idx = best_indices.second;

  if (best_lat_idx < 0 || best_lon_idx < 0) {
    std::cerr << "[LatticePlanner] Fatal: 找不到任何安全的轨迹，需要触发 AEB "
                 "(紧急制动)!"
              << std::endl;
    return false;
  }

  // 1D->2D
  const auto &best_lat = lat_trajs[best_lat_idx];
  const auto &best_lon = lon_trajs[best_lon_idx];

  out_trajectory.clear();
  auto success =
      combine_and_transform_to_2d(best_lat, best_lon, ref_line, out_trajectory);

  return success;
}

std::vector<math::QuinticPolynomial>
LatticePlanner::generate_lateral_trajectories(const VehicleInfo &ego,
                                              const ReferenceLine &ref_line) {
  std::vector<math::QuinticPolynomial> lat_trajs;

  const std::vector<double> target_lat_offset = {3.5, 0, -3.5};
  lat_trajs.reserve(target_lat_offset.size());

  double s0 = 0.0, l0 = 0.0;
  double dl0 = 0.0, ddl0 = 0.0;

  bool is_trasform = ref_line.getFrenetPoint(ego.pose.x, ego.pose.y, s0, l0);

  if (!is_trasform) {
    std::cerr << "[latticePlanner] Error:cartesian to frenet failed!"
              << std::endl;
    return lat_trajs;
  }

  double ref_theta = ref_line.getWayPoint(s0).heading;
  double delta_theta = ego.pose.yaw - ref_theta;

  // 角度归一化 [Pi, -Pi]
  while (delta_theta > M_PI) {
    delta_theta -= 2 * M_PI;
  }

  while (delta_theta < -M_PI) {
    delta_theta += 2 * M_PI;
  }

  dl0 = std::tan(delta_theta);
  ddl0 = 0.0;

  // 纵向探查深度
  double curr_v = ego.v;
  double planning_time = 5.0;
  double min_s = 15.0;
  double total_s = std::max(min_s, curr_v * planning_time);

  for (const double target_l : target_lat_offset) {
    const double l1 = target_l;
    const double dl1 = 0.0;
    const double ddl1 = 0.0;

    lat_trajs.emplace_back(l0, dl0, ddl0, l1, dl1, ddl1, total_s);
  }

  return lat_trajs;
}

std::vector<math::QuinticPolynomial>
LatticePlanner::generate_longitudinal_trajectories(const VehicleInfo &ego) {
  switch (ego.current_state) {
  case pnc_planner::VehicleState::CRUISING:
    return generate_cruise_trajectories(ego);

  case pnc_planner::VehicleState::EMERGENCY:
  case pnc_planner::VehicleState::INIT:
  case pnc_planner::VehicleState::STANDBY:
  default:
    return generate_emergency_trajectories(ego);
  }
}

std::vector<math::QuinticPolynomial>
LatticePlanner::generate_cruise_trajectories(const VehicleInfo &ego) {
  std::vector<math::QuinticPolynomial> lon_cruise_trajs;
}

} // namespace pnc_planner