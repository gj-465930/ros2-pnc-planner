#include "pnc_planner/math/quintic_polynomial.hpp"

namespace pnc_planner {
namespace math {

// clang-format off
QuinticPolynomial::QuinticPolynomial(double x0, double v0, double a0,    
                                     double x1, double v1, double a1, double T)
                                     :T_(T){
  c0_ = x0;
  c1_ = v0;
  c2_ = 0.5 * a0;

  double A = x1 - c0_ - c1_ * T - c2_ * T * T;
  double B = v1 - c1_ - 2 * c2_ * T;
  double C = a1 - 2 * c2_;

  double T2 = T * T;
  double T3 = T * T * T;
  double T4 = T * T * T * T;
  double T5 = T * T * T * T * T;

  c3_ = (1.0 / 2.0) * (20 * A - 8 * B * T + C * T2) / T3;
  c4_ = (-15 * A + 7 * B * T - C * T2) / T4;
  c5_ = (1.0 / 2.0) * (12 * A - 6 * B * T + C * T2) / T5;
}

double QuinticPolynomial::evaluate(double s) const {
  return c0_ + 
         c1_ * s + 
         c2_ * s * s + 
         c3_ * s * s * s + 
         c4_ * s * s * s * s +
         c5_ * s * s * s * s * s;
}

double QuinticPolynomial::evaluate_d(double s) const {
  return c1_ +
         2 * c2_ * s +
         3 * c3_ * s * s +
         4 * c4_ * s * s * s + 
         5 * c5_ * s * s * s * s;
}

double QuinticPolynomial::evaluate_dd(double s) const {
  return 2 * c2_ +
         6 * c3_ * s + 
         12 * c4_ * s * s +
         20 * c5_ * s * s * s;
}

} // namespace math
} // namespace pnc_planner