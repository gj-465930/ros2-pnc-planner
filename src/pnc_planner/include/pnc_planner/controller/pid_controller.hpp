#pragma once

#include "pnc_planner/controller/longitudinal_controller_base.hpp"

namespace pnc_planner ::controller
{

class PidController : public LongitudinalControllerBase
{
public:
  PidController(
    double kp, double ki, double kd, double dt, double max_integral, double max_acc, double min_acc)
  : kp_(kp),
    ki_(ki),
    kd_(kd),
    dt_(dt),
    max_integral_(max_integral),
    max_acc_(max_acc),
    min_acc_(min_acc) {};
  double computeAccel(const Trajectory & traj, const VehicleInfo & ego) override;

private:
  double kp_, ki_, kd_;
  double dt_;

  double max_integral_;  // 防积分饱和阈值
  double max_acc_;
  double min_acc_;

  double previous_error_ = 0.0;
  double integral_ = 0.0;
};

}  // namespace pnc_planner::controller