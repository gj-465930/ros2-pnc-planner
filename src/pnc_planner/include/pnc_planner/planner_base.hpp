/**
 * @file planner_base.hpp
 * @brief 局部规划器抽象基类
 */
#pragma once

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "pnc_planner/ego_vehicle.hpp"
#include "pnc_planner/reference_line.hpp"

namespace pnc_planner {

using Trajectory = std::vector<geometry_msgs::msg::Point>;

class PlannerBase {
public:
  PlannerBase() = default;
  virtual ~PlannerBase() = default;

  virtual bool plan(const EgoVehicle &ego, const ReferenceLine &ref_line,
                    Trajectory &out_trajectory) = 0;
};

} // namespace pnc_planner