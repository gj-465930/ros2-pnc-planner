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

#include "pnc_planner/common.hpp"
#include "pnc_planner/math/quintic_polynomial.hpp"
#include "pnc_planner/planner_base.hpp"

namespace pnc_planner
{

// clang-format off
class LatticePlanner : public PlannerBase {
public:
  explicit LatticePlanner(const LatticePlannerConfig &config) : config_(config), ref_line_(nullptr) {}
  ~LatticePlanner() override = default;

  bool plan(const VehicleInfo &ego, 
            const ReferenceLine &ref_line, 
            Trajectory &out_trajectory) override;

  std::string get_name() const override{
    return "LatticePlanner";
  }

  void setObstacles(const std::vector<Obstacle>& Obstacle){
    obstacles_ = Obstacle;
  }

private:
  LatticePlannerConfig config_;
  std::vector<Obstacle> obstacles_;
  const ReferenceLine *ref_line_;

  //生成横向候选轨迹
  std::vector<math::QuinticPolynomial> generate_lateral_trajectories(
    const VehicleInfo& ego,
    const ReferenceLine& ref_line
  );

  //读取状态机器分发任务
  std::vector<math::QuinticPolynomial> generate_longitudinal_trajectories(
    const VehicleInfo &ego,
    const ReferenceLine &ref_line
  );

  // 生成巡航加减速轨迹
  std::vector<math::QuinticPolynomial> generate_cruise_trajectories(
    const VehicleInfo &ego,
    const ReferenceLine &ref_line
  );

  // 生成紧急刹车轨迹
  std::vector<math::QuinticPolynomial> generate_emergency_trajectories(
    const VehicleInfo &ego,
    const ReferenceLine &ref_line
  );

  std::pair<int, int> evaluate_and_select_best_trajectory(
    const std::vector<math::QuinticPolynomial>& lat_trajs,
    const std::vector<math::QuinticPolynomial>& lon_trajs
  );
  // 碰撞与越界检测
  bool is_trajectory_valid(
    const math::QuinticPolynomial& lat_traj,
    const math::QuinticPolynomial& lon_traj
  );

  // 打分
  double calculate_trajectory_cost(
    const math::QuinticPolynomial &lat_traj,
    const math::QuinticPolynomial &lon_traj
  );

  // 1D转2D
  bool combine_and_transform_to_2d(
    const math::QuinticPolynomial& best_lat,
    const math::QuinticPolynomial& best_lon,
    const ReferenceLine& ref_line,
    Trajectory& out_trajectory
  );
};

} // namespace pnc_planner