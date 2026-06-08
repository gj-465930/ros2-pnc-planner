#pragma once

#include "pnc_planner/controller/lateral_controller_base.hpp"

namespace pnc_planner {
namespace controller {

class StanleyController : public LateralControllerBase {
public:
  StanleyController(double k_gain, double wheelbase)
      : k_(k_gain), wheelbase_(wheelbase) {}

  double computeSteerAngle(const Trajectory &traj,
                           const VehicleInfo &ego) override;

private:
  double k_;
  double wheelbase_;
};

} // namespace controller
} // namespace pnc_planner