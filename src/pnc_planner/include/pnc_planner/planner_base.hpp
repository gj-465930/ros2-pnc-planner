/**
 * @file planner_base.hpp
 * @brief 局部规划器抽象基类
 */
#pragma once

#include "pnc_planner/common.hpp"
#include "pnc_planner/planning/planning_target.hpp"
#include "pnc_planner/reference_line.hpp"

#include <string>
#include <vector>

namespace pnc_planner
{

// clang-format off
class PlannerBase {
public:
  PlannerBase() = default;
  virtual ~PlannerBase() = default;

  virtual bool plan(const VehicleInfo &ego, 
                    const ReferenceLine &ref_line,
                    const planning::PlanningTarget & target,
                    Trajectory &out_trajectory) = 0;

   virtual std::string get_name() const = 0;
};

} // namespace pnc_planner