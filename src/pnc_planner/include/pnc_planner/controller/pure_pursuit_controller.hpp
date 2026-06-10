#pragma once

#include "pnc_planner/controller/lateral_controller_base.hpp"

namespace pnc_planner::controller
{

class PurePursuitController : public LateralControllerBase
{
public:
  PurePursuitController(const double ld_min, const double ld_ratio, const double wheelbase)
  : ld_min_(ld_min), ld_ratio_(ld_ratio), wheelbase_(wheelbase)
  {
  }

  double computeYawRate(const Trajectory & traj, const VehicleInfo & ego) override;

private:
  TrajectoryPoint findLookaheadPoint(const Trajectory & traj, const VehicleInfo & ego);

  double ld_min_;     // 最小前瞻距离
  double ld_ratio_;   // 速度系数
  double wheelbase_;  // 轴距
};
}  // namespace pnc_planner::controller