#include "pnc_planner/lattice_planner.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace pnc_planner
{

bool LatticePlanner::plan(
  const VehicleInfo & ego, const ReferenceLine & ref_line, Trajectory & out_trajectory)
{
  debug_info_ = {};
  out_trajectory.clear();
  ref_line_ = &ref_line;

  // 生成横向候选轨迹
  const auto lat_trajs = generate_lateral_trajectories(ego, ref_line);
  debug_info_.lateral_candidate_count = lat_trajs.size();
  if (lat_trajs.empty()) {
    debug_info_.planning_failure_reason = PlanningFailureReason::LATERAL_GENERATION_FAILED;

    std::cerr << "[LatticePlanner] Error: 横向轨迹生成失败" << std::endl;
    return false;
  }

  // 生成纵向候选轨迹
  const auto lon_trajs = generate_longitudinal_trajectories(ego, ref_line);
  debug_info_.longitudinal_candidate_count = lon_trajs.size();

  if (lon_trajs.empty()) {
    debug_info_.planning_failure_reason = PlanningFailureReason::LONGITUDINAL_GENERATION_FAILED;

    std::cerr << "[LatticePlanner] Error: 纵向轨迹生成失败" << std::endl;
    return false;
  }

  // cost evaluation
  const auto best_indices = evaluate_and_select_best_trajectory(lat_trajs, lon_trajs);
  const int best_lat_idx = best_indices.first;
  const int best_lon_idx = best_indices.second;

  if (best_lat_idx < 0 || best_lon_idx < 0) {
    debug_info_.planning_failure_reason = PlanningFailureReason::NO_VALID_TRAJECTORY;

    std::cerr << "[LatticePlanner] Fatal: 找不到任何安全的轨迹，需要触发 AEB "
                 "(紧急制动)!"
              << std::endl;
    return false;
  }

  // 1D->2D
  const auto & best_lat = lat_trajs[best_lat_idx];
  const auto & best_lon = lon_trajs[best_lon_idx];

  const auto success = combine_and_transform_to_2d(best_lat, best_lon, ref_line, out_trajectory);
  if (!success) {
    debug_info_.planning_failure_reason = PlanningFailureReason::OUTPUT_CONVERSION_FAILED;
  }

  return success;
}

std::vector<math::QuinticPolynomial> LatticePlanner::generate_lateral_trajectories(
  const VehicleInfo & ego, const ReferenceLine & ref_line) const
{
  std::vector<math::QuinticPolynomial> lat_trajs;

  const std::vector<double> target_lat_offset = {3.5, 0, -3.5};
  lat_trajs.reserve(target_lat_offset.size());

  double s0 = 0.0, l0 = 0.0;
  double dl0 = 0.0, ddl0 = 0.0;

  if (const bool is_trasform = ref_line.getFrenetPoint(ego.pose.x, ego.pose.y, s0, l0);
      !is_trasform) {
    std::cerr << "[latticePlanner] Error:cartesian to frenet failed!" << std::endl;
    return lat_trajs;
  }

  const double ref_theta = ref_line.getWayPoint(s0).heading;
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
  const double curr_v = ego.v;
  const double planning_time = config_.planning_time;
  constexpr double min_s = 15.0;
  const double ref_len = ref_line_->getTotalLength() - s0;
  double total_s = std::max(min_s, std::min(curr_v * planning_time, ref_len));

  for (const double target_l : target_lat_offset) {
    const double l1 = target_l;
    constexpr double dl1 = 0.0;
    constexpr double ddl1 = 0.0;

    lat_trajs.emplace_back(l0, dl0, ddl0, l1, dl1, ddl1, total_s);
  }

  return lat_trajs;
}

std::vector<math::QuinticPolynomial> LatticePlanner::generate_longitudinal_trajectories(
  const VehicleInfo & ego, const ReferenceLine & ref_line) const
{
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

std::vector<math::QuinticPolynomial> LatticePlanner::generate_cruise_trajectories(
  const VehicleInfo & ego, const ReferenceLine & ref_line) const
{
  std::vector<math::QuinticPolynomial> lon_cruise_trajs;

  double s0, l0;

  if (const bool is_trasform = ref_line.getFrenetPoint(ego.pose.x, ego.pose.y, s0, l0);
      !is_trasform) {
    std::cerr << "[LatticePlanner] Error: 坐标转换失败";
    return lon_cruise_trajs;
  }

  double v0 = ego.v;
  double a0 = ego.a;

  const double cruise_speed = config_.target_speed;

  const std::vector<double> sample_v = {
    cruise_speed - 2.0, cruise_speed - 1.0, cruise_speed, cruise_speed + 1.0, cruise_speed + 2.0};

  const double T = config_.planning_time;
  const std::vector<double> sample_T = {T - 2.0, T - 1.0, T};

  lon_cruise_trajs.reserve(sample_v.size() * sample_T.size());

  for (const double t : sample_T) {
    for (const double v1 : sample_v) {
      double s1 = s0 + ((v0 + v1) / 2.0) * t;
      double a1 = 0.0;

      if (s1 > ref_line_->getTotalLength()) continue;
      lon_cruise_trajs.emplace_back(s0, v0, a0, s1, v1, a1, t);
    }
  }
  return lon_cruise_trajs;
}

std::vector<math::QuinticPolynomial> LatticePlanner::generate_emergency_trajectories(
  const VehicleInfo & ego, const ReferenceLine & ref_line) const
{
  std::vector<math::QuinticPolynomial> lon_emergency_trajs;

  double s0 = 0.0;
  double l0 = 0.0;

  if (bool is_trasform = ref_line.getFrenetPoint(ego.pose.x, ego.pose.y, s0, l0); !is_trasform) {
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
  const std::vector<double> sample_decel = {-3.0, -5.0, -8.0};

  lon_emergency_trajs.reserve(sample_decel.size());
  for (const double decel : sample_decel) {
    double T = (0 - v0) / decel;

    // 设置下限
    if (T < 0.5) T = 0.5;

    double v1 = 0.0;
    double a1 = 0.0;

    double s1 = s0 + (v0 / 2) * T;
    if (s1 > ref_line_->getTotalLength()) continue;

    lon_emergency_trajs.emplace_back(s0, v0, a0, s1, v1, a1, T);
  }
  return lon_emergency_trajs;
}

std::pair<int, int> LatticePlanner::evaluate_and_select_best_trajectory(
  const std::vector<math::QuinticPolynomial> & lat_trajs,
  const std::vector<math::QuinticPolynomial> & lon_trajs)
{
  // 初始化最小代价
  double min_cost = std::numeric_limits<double>::max();
  int best_lat_idx = -1;
  int best_lon_idx = -1;

  for (size_t i = 0; i < lat_trajs.size(); ++i) {
    for (size_t j = 0; j < lon_trajs.size(); ++j) {
      const auto & lat_traj = lat_trajs[i];
      const auto & lon_traj = lon_trajs[j];

      ++debug_info_.evaluated_pair_count;

      switch (const auto validation_result = is_trajectory_valid(lat_traj, lon_traj)) {
        case TrajectoryValidationResult::VALID:
          ++debug_info_.valid_pair_count;
          break;

        case TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED:
          ++debug_info_.kinematic_rejection_count;
          continue;

        case TrajectoryValidationResult::COLLISION:
          ++debug_info_.collision_rejection_count;
          continue;

        case TrajectoryValidationResult::COORDINATE_CONVERSION_FAILED:
          ++debug_info_.conversion_rejection_count;
          continue;
      }

      if (const double current_cost = calculate_trajectory_cost(lat_traj, lon_traj);
          current_cost < min_cost) {
        min_cost = current_cost;
        best_lat_idx = static_cast<int>(i);
        best_lon_idx = static_cast<int>(j);

        debug_info_.selection_found = true;
        debug_info_.selected_lateral_target = lat_traj.evaluate(lat_traj.get_T());
        debug_info_.selected_duration = lon_traj.get_T();
        debug_info_.selected_cost = current_cost;
      }
    }
  }

  return {best_lat_idx, best_lon_idx};
}

LatticePlanner::TrajectoryValidationResult LatticePlanner::is_trajectory_valid(
  const math::QuinticPolynomial & lat_traj, const math::QuinticPolynomial & lon_traj) const
{
  const double T = lon_traj.get_T();
  constexpr double dt = 0.1;

  const double s0 = lon_traj.evaluate(0.0);

  for (double t = 0.0; t <= T; t += dt) {
    // 纵向有效性判断
    const double v = lon_traj.evaluate_d(t);
    const double a = lon_traj.evaluate_dd(t);
    const double jerk = lon_traj.evaluate_ddd(t);

    if (v < config_.min_v || v > config_.max_v) {
      return TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED;
    }
    if (a < config_.min_acc || a > config_.max_acc) {
      return TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED;
    }
    if (std::abs(jerk) > config_.max_jerk) {
      return TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED;
    }

    // 横向有效性判断
    const double s = lon_traj.evaluate(t);
    const double ds = std::max(s - s0, 0.0);
    const double l = lat_traj.evaluate(ds);

    if (std::abs(l) > config_.max_lat_offset) {
      return TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED;
    }

    // 碰撞检查
    if (!obstacles_.empty()) {
      double x = 0.0, y = 0.0, yaw_ref = 0.0;
      if (!ref_line_->getCartesianPoint(s, l, x, y, yaw_ref)) {
        return TrajectoryValidationResult::COORDINATE_CONVERSION_FAILED;
      }

      for (const auto & obs : obstacles_) {
        const double dx = x - obs.x;
        const double dy = y - obs.y;
        if (const double safe_dist = (3.0 + obs.length) / 2.0;
            std::sqrt(dx * dx + dy * dy) < safe_dist) {
          return TrajectoryValidationResult::COLLISION;
        }
      }
    }
  }

  return TrajectoryValidationResult::VALID;
}

double LatticePlanner::calculate_trajectory_cost(
  const math::QuinticPolynomial & lat_traj, const math::QuinticPolynomial & lon_traj) const
{
  double total_cost = 0.0;

  const double T = lon_traj.get_T();
  constexpr double dt = 0.1;
  const double s0 = lon_traj.evaluate(0);

  double lon_comfort_cost = 0.0;
  double lat_comfort_cost = 0.0;
  double lat_offset_cost = 0.0;

  const double end_v = lon_traj.evaluate_d(T);
  const double speed_diff = config_.target_speed - end_v;

  total_cost += config_.w_speed * (speed_diff * speed_diff);

  for (double t = 0.0; t <= T; t += dt) {
    // 纵向
    const double s = lon_traj.evaluate(t);
    const double a = lon_traj.evaluate_dd(t);
    const double jerk = lon_traj.evaluate_ddd(t);

    lon_comfort_cost += (a * a + jerk * jerk);

    // 横向
    const double ds = std::max(s - s0, 0.0);
    const double l = lat_traj.evaluate(ds);
    const double ddl = lat_traj.evaluate_dd(ds);
    const double dddl = lat_traj.evaluate_ddd(ds);

    lat_comfort_cost += (ddl * ddl + dddl * dddl);
    lat_offset_cost += (l * l);
  }

  total_cost += config_.w_lat * lat_comfort_cost + config_.w_lon * lon_comfort_cost +
                config_.w_offset * lat_offset_cost;

  return total_cost;
}

bool LatticePlanner::combine_and_transform_to_2d(
  const math::QuinticPolynomial & best_lat, const math::QuinticPolynomial & best_lon,
  const ReferenceLine & ref_line, Trajectory & out_trajectory)
{
  out_trajectory.clear();

  const double T = best_lon.get_T();
  constexpr double dt = 0.1;

  const double s0 = best_lon.evaluate(0.0);

  for (double t = 0.0; t <= T; t += dt) {
    // 获取纵向状态
    const double s = best_lon.evaluate(t);
    const double v_lon = best_lon.evaluate_d(t);
    const double a_lon = best_lon.evaluate_dd(t);

    // 获取横向状态
    double ds = s - s0;
    if (ds < 0) ds = 0;

    const double l = best_lat.evaluate(ds);
    const double dl = best_lat.evaluate_d(ds);
    const double ddl = best_lat.evaluate_dd(ds);

    // yaw为在(x, y)下参考线的偏向角
    double x = 0.0, y = 0.0, yaw_ref = 0.0;

    if (const bool is_success = ref_line.getCartesianPoint(s, l, x, y, yaw_ref); !is_success) {
      std::cerr << "[LatticePlanner]投影失败！坐标断裂位于 s = " << s << std::endl;
      out_trajectory.clear();
      return false;
    }

    const double delta_theta = std::atan(dl);
    const double kappa_ref = ref_line.getWayPoint(s).kappa;

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

}  // namespace pnc_planner
