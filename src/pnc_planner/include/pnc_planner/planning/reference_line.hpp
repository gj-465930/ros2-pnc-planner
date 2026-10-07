#pragma once

#include <vector>

#include "pnc_planner/common.hpp"
#include "pnc_planner/math/spline1d.hpp"

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

  math::Spline1D x_spline_;
  math::Spline1D y_spline_;
};

// 参考线生成器
class ReferenceLine {
public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;

  bool init(const std::vector<double> &x, const std::vector<double> &y);
  
  WayPoint getWayPoint(double s) const;

  bool getFrenetPoint(double x, double y, double &s, double &l) const;

  bool getCartesianPoint(double s, double l, double &x, double &y, double &yaw) const;

  double getTotalLength() const { return length_; }

private:
  Spline2D spline_;
  double length_ = 0.0;
};
} // namespace pnc_planner