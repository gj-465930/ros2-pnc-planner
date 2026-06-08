#pragma once

#include "pnc_planner/common.hpp"

namespace pnc_planner {
namespace controller {

/**
 * @brief 纵向控制器基类
 *
 */
class LongitudinalControllerBase {
public:
  virtual ~LongitudinalControllerBase() = default;
  virtual double computeAccel(const Trajectory &traj,
                              const VehicleInfo &ego) = 0;
};

} // namespace controller
} // namespace pnc_planner