/**
 * @file lattice_planner.hpp
 * @author jguo
 * @brief Lattice 网格规划器
 * @version 0.1
 * @date 2026-05-27
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "pnc_planner/math/quintic_polynomial.hpp"
#include "pnc_planner/planning/planner_base.hpp"

#include <cstddef>
#include <cstdint>

namespace pnc_planner
{

struct LatticePlannerConfig
{
  // limits
  double max_v = 0.0;
  double min_v = 0.0;
  double max_acc = 0.0;
  double min_acc = 0.0;
  double max_jerk = 0.0;
  double max_lat_offset = 0.0;
  double target_speed = 0.0;
  double planning_time = 0.0;
  double terminal_safety_decel = 0.0;
  double stop_comfort_decel = 0.0;
  double lateral_transition_distance = 0.0;
  double stop_position_margin = 0.0;

  // weights
  double w_lat = 0.0;
  double w_lon = 0.0;
  double w_offset = 0.0;
  double w_speed = 0.0;
  double w_lateral_target_change = 0.0;

  std::vector<double> lateral_samples = {3.5, 0.0, -3.5};
  std::vector<double> speed_sample_offsets = {-2.0, -1.0, 0.0, 1.0, 2.0};
};

enum class PlanningFailureReason : std::uint8_t {
  NONE = 0,
  INVALID_PLANNING_TARGET,
  LATERAL_GENERATION_FAILED,
  LONGITUDINAL_GENERATION_FAILED,
  NO_VALID_TRAJECTORY,
  OUTPUT_CONVERSION_FAILED
};

struct LatticePlannerDebugInfo
{
  std::size_t lateral_candidate_count = 0;
  std::size_t longitudinal_candidate_count = 0;
  std::size_t evaluated_pair_count = 0;
  std::size_t valid_pair_count = 0;
  std::size_t kinematic_rejection_count = 0;
  std::size_t velocity_rejection_count = 0;
  std::size_t acceleration_rejection_count = 0;
  std::size_t jerk_rejection_count = 0;
  std::size_t conversion_rejection_count = 0;
  std::size_t collision_rejection_count = 0;
  std::size_t terminal_safety_rejection_count = 0;

  std::vector<Trajectory> valid_candidate_trajectories;

  bool selection_found = false;
  double selected_lateral_target = 0.0;
  double selected_duration = 0.0;
  double selected_cost = 0.0;

  // 最小拒绝jerk已经对应的_T()以及对应的S
  double minimum_rejected_max_jerk = 0.0;
  double corresponding_duration = 0.0;
  // 这条拒绝的最小jerk走过的最远距离（由于五次多项式的特性所以不一定是终点）
  double maximum_s_for_minimum_rejected_jerk = 0.0;
  // 超过stop_s的最少候选
  double minimum_terminal_overshoot = 0.0;
  double corresponding_terminal_duration = 0.0;
  double corresponding_terminal_rejected_max_jerk = 0.0;  // 这条轨迹出现的最大jerk

  // tagrt -> STOP debug信息
  double stop_start_s = 0.0;
  double stop_start_v = 0.0;
  double stop_start_a = 0.0;
  double stop_target_s = 0.0;
  double stop_remaining_distance = 0.0;
  double stop_min_duration = 0.0;
  double stop_max_duration = 0.0;

  PlanningFailureReason planning_failure_reason = PlanningFailureReason::NONE;
};

// clang-format off
class LatticePlanner : public PlannerBase {
public:
  explicit LatticePlanner(const LatticePlannerConfig &config) : config_(config), ref_line_(nullptr) {}
  ~LatticePlanner() override = default;

  bool plan(const VehicleInfo &ego, 
            const ReferenceLine &ref_line,
            const planning::PlanningTarget &target,
            Trajectory &out_trajectory) override;

  std::string get_name() const override{
    return "LatticePlanner";
  }

  void setObstacles(const std::vector<Obstacle> & obstacles)
  {
    obstacles_ = obstacles;
  }

  const LatticePlannerDebugInfo & getLastDebugInfo() const
  {
    return debug_info_;
  }

private:
  LatticePlannerConfig config_;
  std::vector<Obstacle> obstacles_;
  const ReferenceLine *ref_line_;
  LatticePlannerDebugInfo debug_info_;

  bool has_previous_lateral_target_ = false;
  double previous_lateral_target_ = 0.0;

  enum class TrajectoryValidationResult : std::uint8_t
  {
    VALID = 0,
    VELOCITY_CONSTRAINT_VIOLATED,
    ACCELERATION_CONSTRAINT_VIOLATED,
    JERK_CONSTRAINT_VIOLATED,
    COORDINATE_CONVERSION_FAILED,
    LATERAL_OFFSET_CONSTRAINT_VIOLATED,
    COLLISION,
    UNSAFE_TERMINAL_STATE
  };

  //生成横向候选轨迹
  std::vector<math::QuinticPolynomial> generate_lateral_trajectories(
    const VehicleInfo& ego,
    const ReferenceLine& ref_line
  ) const;

  //读取状态机器分发任务
  std::vector<math::QuinticPolynomial> generate_longitudinal_trajectories(
    const VehicleInfo &ego,
    const ReferenceLine &ref_line,
    const planning::PlanningTarget &target
  ) ;

  // 生成巡航加减速轨迹
  std::vector<math::QuinticPolynomial> generate_cruise_trajectories(
    const VehicleInfo &ego,
    const ReferenceLine &ref_line,
    const planning::PlanningTarget &target
  ) const;

  // 生成停止轨迹
  std::vector<math::QuinticPolynomial> generate_stop_trajectories(
    const VehicleInfo &ego,
    const ReferenceLine &ref_line,
    const planning::PlanningTarget &target);

  std::pair<int, int> evaluate_and_select_best_trajectory(
    const std::vector<math::QuinticPolynomial>& lat_trajs,
    const std::vector<math::QuinticPolynomial>& lon_trajs,
    const planning::PlanningTarget &target
  );
  // 碰撞与越界检测
  TrajectoryValidationResult is_trajectory_valid(
    const math::QuinticPolynomial& lat_traj,
    const math::QuinticPolynomial& lon_traj,
    const planning::PlanningTarget &target
  ) const;

  // 打分
  double calculate_trajectory_cost(
    const math::QuinticPolynomial &lat_traj,
    const math::QuinticPolynomial &lon_traj,
    const planning::PlanningTarget &target
  ) const;

  // 1D转2D
  static bool combine_and_transform_to_2d(
    const math::QuinticPolynomial& best_lat,
    const math::QuinticPolynomial& best_lon,
    const ReferenceLine& ref_line,
    Trajectory& out_trajectory
  );
};

} // namespace pnc_planner