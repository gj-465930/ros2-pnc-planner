#pragma once

#include "pnc_planner/controller/longitudinal_controller_base.hpp"

namespace pnc_planner {
namespace controller {

class PidController : public LongitudinalControllerBase {
public:
  PidController();
  double computeAccel(const Trajectory &traj, const VehicleInfo &ego) override;
};
} // namespace controller
} // namespace pnc_planner