#pragma once

#include "pnc_planner/common.hpp"

namespace pnc_planner ::controller
{

class LateralControllerBase
{
public:
  LateralControllerBase() = default;
  virtual ~LateralControllerBase() = default;

  virtual double computeSteerAngle(const Trajectory & traj, const VehicleInfo & ego) = 0;
};

}  // namespace pnc_planner::controller