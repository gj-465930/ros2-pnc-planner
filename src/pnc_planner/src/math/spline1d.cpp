#include "pnc_planner/math/spline1d.hpp"

#include <algorithm>

namespace pnc_planner ::math
{

bool Spline1D::init(const std::vector<double> & s, const std::vector<double> & y)
{
  if (s.size() != y.size() || s.size() < 3) return false;

  int n = s.size();
  s_ = s;
  y_ = y;
  a_.resize(n - 1);
  b_.resize(n - 1);
  c_.resize(n - 1);
  d_.resize(n - 1);

  // 使用Thomas 算法求解三对角矩阵
  std::vector<double> h(n - 1);  // 存放每条路径的对应的弧长
  std::vector<double> d(n - 1);  // 存放d
  for (int i = 0; i < n - 1; ++i) {
    h[i] = s[i + 1] - s[i];
    d[i] = (y[i + 1] - y[i]) / h[i];
  }

  std::vector<double> lower(n, 0.0), main_diag(n, 1.0), upper(n, 0.0), B(n, 0.0);
  // 三对角矩阵A
  for (int i = 1; i < n - 1; ++i) {
    lower[i] = h[i - 1];
    main_diag[i] = 2.0 * (h[i - 1] + h[i]);
    upper[i] = h[i];
    B[i] = 6.0 * (d[i] - d[i - 1]);
  }

  // 追赶法——追
  std::vector<double> c_prime(n, 0.0), d_prime(n, 0.0), M(n, 0.0);
  for (int i = 1; i < n; ++i) {
    double m = 1.0 / (main_diag[i] - lower[i] * c_prime[i - 1]);
    c_prime[i] = upper[i] * m;
    d_prime[i] = (B[i] - d_prime[i - 1] * lower[i]) * m;
  }

  // 追赶法——赶
  M[n - 1] = d_prime[n - 1];
  for (int i = n - 2; i >= 0; --i) {
    M[i] = d_prime[i] - c_prime[i] * M[i + 1];
  }

  // 根据曲率反推多项式系数
  for (int i = 0; i < n - 1; ++i) {
    a_[i] = (M[i + 1] - M[i]) / (6.0 * h[i]);
    b_[i] = M[i] * 0.5;
    c_[i] = d[i] - h[i] * (2.0 * M[i] + M[i + 1]) / 6.0;
    d_[i] = y[i];
  }

  return true;
}

double Spline1D::calc(double s) const
{
  // 先查询查询的值处于哪条曲线
  int idx = Spline1D::searchIndex(s);

  double ds = s - s_[idx];

  return a_[idx] * ds * ds * ds + b_[idx] * ds * ds + c_[idx] * ds + d_[idx];
}

double Spline1D::calcDerivative(double s) const
{
  int idx = Spline1D::searchIndex(s);

  double ds = s - s_[idx];

  return 3.0 * a_[idx] * ds * ds + 2.0 * b_[idx] * ds + c_[idx];
}

double Spline1D::calcSecondDerivative(double s) const
{
  int idx = Spline1D::searchIndex(s);

  double ds = s - s_[idx];

  return 6.0 * a_[idx] * ds + 2.0 * b_[idx];
}

int Spline1D::searchIndex(double s) const
{
  if (s_.empty()) return 0;
  // 边界拦截
  if (s <= s_.front()) {
    return 0;
  }

  if (s >= s_.back()) {
    return s_.size() - 2;
  }

  auto it = std::upper_bound(s_.begin(), s_.end(), s);

  return std::distance(s_.begin(), it) - 1;
}

}  // namespace pnc_planner::math