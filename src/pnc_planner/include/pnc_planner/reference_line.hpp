#pragma once

#include <memory>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pnc_planner/common.hpp"

// 二维样条插值器

namespace pnc_planner {

class Spline2D {
public:
  Spline2D() = default;

  // 内部进行矩阵求解
  bool init(const std::vector<double> &x, const std::vector<double> &y);

  std::vector<double> calcPosition(double s);

  double calcHeading(double s);

  double calcKappa(double s);

  double getTotalLength() const;

private:
  std::vector<double> s_;
  std::vector<double> x_;
  std::vector<double> y_;
};

// 参考线生成器
class ReferenceLine {
public:
  explicit ReferenceLine(rclcpp::Node::SharedPtr node);

private:
  rclcpp::Node::SharedPtr node_;
};
} // namespace pnc_planner