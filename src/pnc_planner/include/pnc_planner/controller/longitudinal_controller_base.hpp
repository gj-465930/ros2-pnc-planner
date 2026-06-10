#pragma once

#include "pnc_planner/common.hpp"

namespace pnc_planner ::controller
{

/**
 * @brief 纵向控制器基类
 *
 */
class LongitudinalControllerBase
{
public:
  virtual ~LongitudinalControllerBase() = default;
  virtual double computeAccel(const Trajectory & traj, const VehicleInfo & ego) = 0;
};

}  // namespace pnc_planner::controller