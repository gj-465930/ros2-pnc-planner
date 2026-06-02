#include <algorithm>
#include <cmath>
#include <iostream>

#include "pnc_planner/lattice_planner.hpp"

namespace pnc_planner {

bool LatticePlanner::plan(const VehicleInfo &ego, const ReferenceLine &ref_line,
                          Trajectory &out_trajectory) {
  ref_line_ = &ref_line;

  // 生成横向候选轨迹
  auto lat_trajs = generate_lateral_trajectories(ego, ref_line);
  if (lat_trajs.empty()) {
    std::cerr << "[LatticePlanner] Error: 横向轨迹生成失败" << std::endl;
    return false;
  }

  // 生成纵向候选轨迹
  auto lon_trajs = generate_longitudinal_trajectories(ego, ref_line);
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
LatticePlanner::generate_longitudinal_trajectories(
    const VehicleInfo &ego, const ReferenceLine &ref_line) {
  switch (ego.current_state) {
  case pnc_planner::VehicleState::CRUISING:
    return generate_cruise_trajectories(ego, ref_line);

  case pnc_planner::VehicleState::EMERGENCY:
  case pnc_planner::VehicleState::INIT:
  case pnc_planner::VehicleState::STANDBY:
  default:
    return generate_emergency_trajectories(ego, ref_line);
  }
}

std::vector<math::QuinticPolynomial>
LatticePlanner::generate_cruise_trajectories(const VehicleInfo &ego,
                                             const ReferenceLine &ref_line) {
  std::vector<math::QuinticPolynomial> lon_cruise_trajs;

  double s0, l0;
  bool is_trasform = ref_line.getFrenetPoint(ego.pose.x, ego.pose.y, s0, l0);

  if (!is_trasform) {
    std::cerr << "[LatticePlanner] Error: 坐标转换失败";
    return lon_cruise_trajs;
  }

  double v0 = ego.v;
  double a0 = ego.a;

  double cruise_speed = 15.0;

  std::vector<double> sample_v = {cruise_speed, cruise_speed - 1.0,
                                  cruise_speed + 1.0};
  std::vector<double> sample_T = {3.0, 4.0, 5.0};

  lon_cruise_trajs.reserve(sample_v.size() * sample_T.size());

  for (const double T : sample_T) {
    for (const double v1 : sample_v) {
      double s1 = s0 + ((v0 + v1) / 2.0) * T;
      double a1 = 0.0;

      lon_cruise_trajs.emplace_back(s0, v0, a0, s1, v1, a1, T);
    }
  }
  return lon_cruise_trajs;
}

std::vector<math::QuinticPolynomial>
LatticePlanner::generate_emergency_trajectories(const VehicleInfo &ego,
                                                const ReferenceLine &ref_line) {
  std::vector<math::QuinticPolynomial> lon_emergency_trajs;

  double s0 = 0.0;
  double l0 = 0.0;

  bool is_trasform = ref_line.getFrenetPoint(ego.pose.x, ego.pose.y, s0, l0);

  if (!is_trasform) {
    std::cerr << "[LatticePlanner] Error: 停车规划s0获取失败" << std::endl;
    return lon_emergency_trajs;
  }

  double v0 = ego.v;
  double a0 = ego.a;

  // INIT/STANDBY 状态
  if (v0 < 0.1) {
    double T = 3.0;
    lon_emergency_trajs.emplace_back(s0, 0.0, 0.0, s0, 0.0, 0.0, T);
    return lon_emergency_trajs;
  }

  // 刹车
  std::vector<double> sample_decel = {-3.0, -5.0, -8.0};

  lon_emergency_trajs.reserve(sample_decel.size());
  for (const double decel : sample_decel) {
    double T = (0 - v0) / decel;

    // 设置下限
    if (T < 0.5)
      T = 0.5;

    double v1 = 0.0;
    double a1 = 0.0;

    double s1 = s0 + (v0 / 2) * T;

    lon_emergency_trajs.emplace_back(s0, v0, a0, s1, v1, a1, T);
  }
  return lon_emergency_trajs;
}

std::pair<int, int> LatticePlanner::evaluate_and_select_best_trajectory(
    const std::vector<math::QuinticPolynomial> &lat_trajs,
    const std::vector<math::QuinticPolynomial> &lon_trajs) {

  // 初始化最小代价
  double min_cost = std::numeric_limits<double>::max();
  int best_lat_idx = -1;
  int best_lon_idx = -1;

  for (size_t i = 0; i < lat_trajs.size(); ++i) {
    for (size_t j = 0; j < lon_trajs.size(); ++j) {

      const auto &lat_traj = lat_trajs[i];
      const auto &lon_traj = lon_trajs[j];

      if (!is_trajectory_valid(lat_traj, lon_traj)) {
        continue;
      }

      double current_cost = calculate_trajectory_cost(lat_traj, lon_traj);

      if (current_cost < min_cost) {
        min_cost = current_cost;
        best_lat_idx = static_cast<int>(i);
        best_lon_idx = static_cast<int>(j);
      }
    }
  }

  return {best_lat_idx, best_lon_idx};
}

bool LatticePlanner::is_trajectory_valid(
    const math::QuinticPolynomial &lat_traj,
    const math::QuinticPolynomial &lon_traj) {

  double T = lon_traj.get_T();
  double dt = 0.1;

  double s0 = lon_traj.evaluate(0.0);

  // clang-format off
  for (double t = 0.0; t <= T; t += dt) {
    // 纵向有效性判断
    double v = lon_traj.evaluate_d(t);
    double a = lon_traj.evaluate_dd(t);
    double jerk = lon_traj.evaluate_ddd(t);

    if (v < config_.min_v || v > config_.max_v) return false;
    if (a < config_.min_acc || a > config_.max_acc) return false;
    if (std::abs(jerk) > config_.max_jerk) return false;

    // 横向有效性判断
    double s = lon_traj.evaluate(t);
    double ds = s - s0;

    if(ds > lat_traj.get_T()) return false;
    if(ds < 0.0) ds = 0.0;

    double l = lat_traj.evaluate(ds);

    if(std::abs(l) > config_.max_lat_offset) return false;

    // 碰撞检查
    if(!obstacles_.empty()){
      double x = 0.0, y = 0.0, yaw_ref = 0.0;
      ref_line_->getCartesianPoint(s, l, x, y,yaw_ref);

      for(const auto &obs : obstacles_){
        double dx = x - obs.x;
        double dy = y - obs.y;
        double safe_dist = (3.0 + obs.length) / 2.0;
        if(std::sqrt(dx * dx + dy * dy) < safe_dist) return false;
      }
    }
  }
  // clang-format on

  return true;
}

double LatticePlanner::calculate_trajectory_cost(
    const math::QuinticPolynomial &lat_traj,
    const math::QuinticPolynomial &lon_traj) {}

bool LatticePlanner::combine_and_transform_to_2d(
    const math::QuinticPolynomial &best_lat,
    const math::QuinticPolynomial &best_lon, const ReferenceLine &ref_line,
    Trajectory &out_trajectory) {

  out_trajectory.clear();

  double T = best_lon.get_T();
  double dt = 0.1;

  double s0 = best_lon.evaluate(0.0);

  for (double t = 0.0; t <= T; t += dt) {
    // 获取纵向状态
    double s = best_lon.evaluate(t);
    double v_lon = best_lon.evaluate_d(t);
    double a_lon = best_lon.evaluate_dd(t);

    // 获取横向状态
    double ds = s - s0;
    if (ds < 0)
      ds = 0;

    double l = best_lat.evaluate(ds);
    double dl = best_lat.evaluate_d(ds);
    double ddl = best_lat.evaluate_dd(ds);

    // yaw为在(x, y)下参考线的偏向角
    double x = 0.0, y = 0.0, yaw_ref = 0.0;

    bool is_success = ref_line.getCartesianPoint(s, l, x, y, yaw_ref);

    if (!is_success) {
      std::cerr << "[LatticePlanner]投影失败！坐标断裂位于 s = " << s
                << std::endl;
      out_trajectory.clear();
      return false;
    }

    double delta_theta = std::atan(dl);
    double kappa_ref = ref_line.getWayPoint(s).kappa;

    TrajectoryPoint pt;
    pt.x = x;
    pt.y = y;
    pt.heading = yaw_ref + delta_theta;
    pt.kappa = ddl + kappa_ref;

    pt.v = v_lon;
    pt.a = a_lon;

    out_trajectory.push_back(pt);
  }

  return true;
}

} // namespace pnc_planner