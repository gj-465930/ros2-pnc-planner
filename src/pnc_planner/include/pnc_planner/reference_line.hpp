#pragma once

#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pnc_planner/common.hpp"
#include "pnc_planner/math_utils/spline1d.hpp"

// 二维样条插值器

namespace pnc_planner {

class Spline2D {
public:
  Spline2D() = default;

  // 内部进行矩阵求解
  bool init(const std::vector<double> &x, const std::vector<double> &y);

  std::vector<double> calcPosition(double s) const;

  double calcHeading(double s) const;

  double calcKappa(double s) const;

  double getTotalLength() const;

private:
  std::vector<double> s_;

  math_utils::Spline1D x_spline_;
  math_utils::Spline1D y_spline_;
};

// 参考线生成器
class ReferenceLine {
public:
  explicit ReferenceLine(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
};
} // namespace pnc_planner