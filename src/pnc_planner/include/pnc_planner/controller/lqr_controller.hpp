#pragma once

#include "pnc_planner/controller/lateral_controller_base.hpp"

namespace pnc_planner ::controller
{

class LqrController : public LateralControllerBase
{
public:
  LqrController();

  double computeSteerAngle(const Trajectory & traj, const VehicleInfo & ego) override;
};

}  // namespace pnc_planner::controller