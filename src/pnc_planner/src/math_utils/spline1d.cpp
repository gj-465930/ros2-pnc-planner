#include "pnc_planner/math_utils/spline1d.hpp"

namespace pnc_planner {
namespace math_utils {

bool Spline1D::init(const std::vector<double> &s,
                    const std::vector<double> &y) {
  if (s.size() != y.size() || s.size() < 3)
    return false;

  int n = s.size();
  s_ = s;
  y_ = y;
  a_.resize(n - 1);
  b_.resize(n - 1);
  c_.resize(n - 1);
  d_.resize(n - 1);

  // 使用Thomas 算法求解三对角矩阵
  
}

double Spline1D::calc(double) const {}

double Spline1D::calcDerivative(double s) const {}

double Spline1D::calcSecondDerivative(double) const {}

} // namespace math_utils
} // namespace pnc_planner