#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

// Affine
Eigen::MatrixXd translate_then_rotate(
  std::vector<double> & x, std::vector<double> & y,
  double offset_x, double offset_y, double angle) {

  size_t sz = x.size();

  Eigen::MatrixXd translated(2, sz);
  translated.row(0) = Eigen::Map<Eigen::VectorXd>(&x[0], sz) + Eigen::VectorXd::Ones(sz) * offset_x;
  translated.row(1) = Eigen::Map<Eigen::VectorXd>(&y[0], sz) + Eigen::VectorXd::Ones(sz) * offset_y;

  Eigen::MatrixXd rotator(2, 2);
  rotator << cos(angle), -sin(angle),
             sin(angle), cos(angle);

  return rotator * translated;
}

// // Evaluate a polynomial.
// double polyeval(const Eigen::VectorXd & coeffs, double x) {
//   double result = 0.0;
//   for (int i = 0; i < coeffs.size(); i++) {
//     result += coeffs[i] * pow(x, i);
//   }
//   return result;
// }

// // Evaluate d/dx of a polynomial at given x.
// double dydx(const Eigen::VectorXd & coeffs, double x) {
//   double result = 0.0;
//   for (int i = 1; i < coeffs.size(); i++) {
//     result += i * coeffs[i] * pow(x, i - 1);
//   }
//   return result;
// }

std::vector<double> global_kinetic_model(
  const std::vector<double> & state,
  double steering, double throttle, double dt, double Lf) {

  double px = state[0];
  double py = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // double cte = polyeval(coeffs, px) - py;
  // double epsi = psi - atan(dydx(coeffs, px));

  double helper_psi_term = v / Lf * steering * dt;

  double next_px = px + v * cos(psi) * dt;
  double next_py = py + v * sin(psi) * dt;
  double next_psi = psi + helper_psi_term;
  double next_v = v + throttle * dt;
  double next_cte = cte + v * sin(epsi) * dt;
  double next_epsi = epsi + helper_psi_term;

  return std::vector<double> {next_px, next_py, next_psi, next_v, next_cte, next_epsi};
}

std::vector<double> eigen_to_std_vector(Eigen::VectorXd eigen) {
  auto begin = eigen.data();
  return std::vector<double>(begin, begin + eigen.size());
}

#endif /* TOOLS_H */
