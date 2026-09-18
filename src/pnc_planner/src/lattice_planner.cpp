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

  previous_lateral_target_ = best_lat.evaluate(best_lat.get_T());
  has_previous_lateral_target_ = true;

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
  const double ref_len = ref_line.getTotalLength() - s0;
  if (ref_len <= 0.0) {
    return lat_trajs;
  }

  if (
    !std::isfinite(config_.lateral_transition_distance) ||
    config_.lateral_transition_distance <= 0.0) {
    return lat_trajs;
  }

  const double total_s = std::min(config_.lateral_transition_distance, ref_len);

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

  const double planning_time = config_.planning_time;

  lon_cruise_trajs.reserve(sample_v.size());

  for (const double v1 : sample_v) {
    const double s1 = s0 + ((v0 + v1) / 2.0) * planning_time;
    constexpr double a1 = 0.0;

    if (s1 > ref_line.getTotalLength()) {
      continue;
    }

    lon_cruise_trajs.emplace_back(s0, v0, a0, s1, v1, a1, planning_time);
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

        case TrajectoryValidationResult::UNSAFE_TERMINAL_STATE:
          ++debug_info_.terminal_safety_rejection_count;
          continue;
      }

      double current_cost = calculate_trajectory_cost(lat_traj, lon_traj);
      const double lateral_target = lat_traj.evaluate(lat_traj.get_T());

      // 加入偏移权重
      if (has_previous_lateral_target_) {
        const double target_change = lateral_target - previous_lateral_target_;
        current_cost += config_.w_lateral_target_change * target_change * target_change;
      }

      if (current_cost < min_cost) {
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
  constexpr double constraint_tolerance = 1e-6;

  const double T = lon_traj.get_T();
  constexpr double dt = 0.1;

  const double s0 = lon_traj.evaluate(0.0);

  for (double t = 0.0; t <= T; t += dt) {
    // 纵向有效性判断
    const double v = lon_traj.evaluate_d(t);
    const double a = lon_traj.evaluate_dd(t);
    const double jerk = lon_traj.evaluate_ddd(t);

    if (v < config_.min_v - constraint_tolerance || v > config_.max_v + constraint_tolerance) {
      return TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED;
    }
    if (a < config_.min_acc - constraint_tolerance || a > config_.max_acc + constraint_tolerance) {
      return TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED;
    }
    if (std::abs(jerk) > config_.max_jerk + constraint_tolerance) {
      return TrajectoryValidationResult::KINEMATIC_CONSTRAINT_VIOLATED;
    }

    // 横向有效性判断
    const double s = lon_traj.evaluate(t);
    const double lateral_progress = std::clamp(s - s0, 0.0, lat_traj.get_T());
    const double l = lat_traj.evaluate(lateral_progress);

    if (std::abs(l) > config_.max_lat_offset + constraint_tolerance) {
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

  if (!std::isfinite(config_.terminal_safety_decel) || config_.terminal_safety_decel <= 0.0) {
    return TrajectoryValidationResult::UNSAFE_TERMINAL_STATE;
  }

  const double terminal_s = lon_traj.evaluate(T);
  const double terminal_v = std::max(lon_traj.evaluate_d(T), 0.0);

  const double braking_distance = terminal_v * terminal_v / (2.0 * config_.terminal_safety_decel);

  const double terminal_progress = std::clamp(terminal_s - s0, 0.0, lat_traj.get_T());
  const double terminal_l = lat_traj.evaluate(terminal_progress);

  if (const double available_distance = std::max(ref_line_->getTotalLength() - terminal_s, 0.0);
      braking_distance > available_distance) {
    return TrajectoryValidationResult::UNSAFE_TERMINAL_STATE;
  }

  if (obstacles_.empty()) {
    return TrajectoryValidationResult::VALID;
  }

  if (const double checked_distance = braking_distance; checked_distance > 0.0) {
    constexpr double braking_check_step = 0.1;
    const auto sample_count =
      static_cast<std::size_t>(std::ceil(checked_distance / braking_check_step));

    for (std::size_t index = 1; index <= sample_count; ++index) {
      const double ratio = static_cast<double>(index) / static_cast<double>(sample_count);
      const double braking_s = terminal_s + checked_distance * ratio;

      double x = 0.0;
      double y = 0.0;

      if (double yaw_ref = 0.0;
          !ref_line_->getCartesianPoint(braking_s, terminal_l, x, y, yaw_ref)) {
        return TrajectoryValidationResult::COORDINATE_CONVERSION_FAILED;
      }

      for (const auto & obs : obstacles_) {
        const double dx = x - obs.x;
        const double dy = y - obs.y;

        if (const double safe_dist = (3.0 + obs.length) / 2.0;
            std::sqrt(dx * dx + dy * dy) < safe_dist) {
          return TrajectoryValidationResult::UNSAFE_TERMINAL_STATE;
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
    // 如果横向的规划距离比纵向的长那么到达目标点朝向一定不是0
    // 反之如果横向规划距离比纵向规划距离短那么到达目标点的时候ego是直走的
    const double raw_lateral_progress = std::max(s - s0, 0.0);
    const bool lateral_motion_complete = raw_lateral_progress >= lat_traj.get_T();
    const double lateral_progress = std::min(raw_lateral_progress, lat_traj.get_T());
    const double l = lat_traj.evaluate(lateral_progress);
    const double ddl = lateral_motion_complete ? 0.0 : lat_traj.evaluate_dd(lateral_progress);
    const double dddl = lateral_motion_complete ? 0.0 : lat_traj.evaluate_ddd(lateral_progress);

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
    const double raw_lateral_progress = std::max(s - s0, 0.0);
    const double lateral_progress = std::min(raw_lateral_progress, best_lat.get_T());
    const bool lateral_motion_complete = raw_lateral_progress >= best_lat.get_T();
    const double l = best_lat.evaluate(lateral_progress);
    const double dl = lateral_motion_complete ? 0.0 : best_lat.evaluate_d(lateral_progress);
    const double ddl = lateral_motion_complete ? 0.0 : best_lat.evaluate_dd(lateral_progress);

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
