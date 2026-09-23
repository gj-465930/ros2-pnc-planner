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

#include "pnc_planner/common.hpp"
#include "pnc_planner/math/quintic_polynomial.hpp"
#include "pnc_planner/planner_base.hpp"

#include <cstddef>
#include <cstdint>

namespace pnc_planner
{

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
  std::size_t conversion_rejection_count = 0;
  std::size_t collision_rejection_count = 0;
  std::size_t terminal_safety_rejection_count = 0;

  std::vector<Trajectory> valid_candidate_trajectories;

  bool selection_found = false;
  double selected_lateral_target = 0.0;
  double selected_duration = 0.0;
  double selected_cost = 0.0;

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
    KINEMATIC_CONSTRAINT_VIOLATED,
    COORDINATE_CONVERSION_FAILED,
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
  ) const;

  // 生成巡航加减速轨迹
  std::vector<math::QuinticPolynomial> generate_cruise_trajectories(
    const VehicleInfo &ego,
    const ReferenceLine &ref_line,
    const planning::PlanningTarget &target
  ) const;

  std::pair<int, int> evaluate_and_select_best_trajectory(
    const std::vector<math::QuinticPolynomial>& lat_trajs,
    const std::vector<math::QuinticPolynomial>& lon_trajs,
    const planning::PlanningTarget &target
  );
  // 碰撞与越界检测
  TrajectoryValidationResult is_trajectory_valid(
    const math::QuinticPolynomial& lat_traj,
    const math::QuinticPolynomial& lon_traj
  ) const;

  // 打分
  double calculate_trajectory_cost(
    const math::QuinticPolynomial &lat_traj,
    const math::QuinticPolynomial &lon_traj,
    const planning::PlanningTarget &target
  ) const;

  // 1D转2D
  bool combine_and_transform_to_2d(
    const math::QuinticPolynomial& best_lat,
    const math::QuinticPolynomial& best_lon,
    const ReferenceLine& ref_line,
    Trajectory& out_trajectory
  );
};

} // namespace pnc_planner