#pragma once

#include <vector>

namespace pnc_planner ::math
{

class Spline1D
{
public:
  Spline1D() = default;
  ~Spline1D() = default;

  bool init(const std::vector<double> & s, const std::vector<double> & y);

  // 输出差值后的y
  double calc(double s) const;

  // 计算一阶导数
  double calcDerivative(double s) const;

  // 计算二阶导数
  double calcSecondDerivative(double s) const;

private:
  // 查询当前弧长在对应的哪段曲线的值
  int searchIndex(double s) const;

  std::vector<double> s_;
  std::vector<double> y_;

  // 三次多项式系数
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

}  // namespace pnc_planner::math