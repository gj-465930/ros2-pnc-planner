#pragma once

#include "pnc_planner/controller/lateral_controller_base.hpp"

namespace pnc_planner {
namespace controller {

class LqrController : public LateralControllerBase {
public:
  LqrController();

  double computeSteerAngle(const Trajectory &traj,
                           const VehicleInfo &ego) override;
};
} // namespace controller
} // namespace pnc_planner