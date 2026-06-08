#pragma once

#include "pnc_planner/controller/lateral_controller_base.hpp"

namespace pnc_planner {

namespace controller {

class PurePursuitController : public LateralControllerBase {
public:
  PurePursuitController(double lookahead, double wheelbase)
      : lookahead_(lookahead), wheelbase_(wheelbase) {}

  double computeSteerAngle(const Trajectory &traj,
                           const VehicleInfo &ego) override;

private:
  TrajectoryPoint findLookaheadPoint(const Trajectory &traj,
                                     const VehicleInfo &ego, double ld);

  double lookahead_; // 前瞻距离
  double wheelbase_; // 轴距
};
} // namespace controller
} // namespace pnc_planner