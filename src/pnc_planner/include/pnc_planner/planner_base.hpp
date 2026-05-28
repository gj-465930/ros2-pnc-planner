/**
 * @file planner_base.hpp
 * @brief 局部规划器抽象基类
 */
#pragma once

#include <string>
#include <vector>

#include "pnc_planner/common.hpp"
#include "pnc_planner/reference_line.hpp"

namespace pnc_planner {

// clang-format off
class PlannerBase {
public:
  PlannerBase() = default;
  virtual ~PlannerBase() = default;

  virtual bool plan(const VehicleInfo &ego, 
                    const ReferenceLine &ref_line, 
                    Trajectory &out_trajectory) = 0;

   virtual std::string get_name() const = 0;
};

} // namespace pnc_planner