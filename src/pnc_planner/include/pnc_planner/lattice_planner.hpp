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

#include "pnc_planner/math/quintic_polynomial.hpp"
#include "pnc_planner/planner_base.hpp"

namespace pnc_planner {

// clang-format off
class LatticePlanner : public PlannerBase {
public:
  LatticePlanner() = default;
  ~LatticePlanner() override = default;

  bool plan(const VehicleInfo &ego, 
            const ReferenceLine &ref_line, 
            Trajectory &out_trajectory) override;

  std::string get_name() const override{
    return "LatticePlanner";
  }

private:
  /**
   * @brief 生成横向候选轨迹
   * 
   * @return std::vector<math::QuinticPolynomial> 
   */
  std::vector<math::QuinticPolynomial> generate_lateral_trajectories(
    const VehicleInfo& ego,
    const ReferenceLine& ref_line
  );

  /**
   * @brief 生成纵向候选轨迹
   * 
   * @param ego 
   * @return std::vector<math::QuinticPolynomial> 
   */
  std::vector<math::QuinticPolynomial> generate_longitudinal_trajectories(
    const VehicleInfo& ego
  );

  /**
   * @brief cost_func
   * 
   * @param lat_trajs  横向候选集 
   * @param lon_trajs  纵向候选集
   * @return std::pair<int, int> 
   */
  std::pair<int, int> evaluate_and_select_best_trajectory(
    const std::vector<math::QuinticPolynomial>& lat_trajs,
    const std::vector<math::QuinticPolynomial>& lon_trajs
  );

  /**
   * @brief 1D映射到2D
   * 
   * @param best_lat 
   * @param best_lon 
   * @param ref_line 
   * @param out_trajectory 
   * @return true 
   * @return false 
   */
  bool combine_and_transform_to_2d(
    const math::QuinticPolynomial& best_lat,
    const math::QuinticPolynomial& best_lon,
    const ReferenceLine& ref_line,
    Trajectory& out_trajectory
  );
};

} // namespace pnc_planner