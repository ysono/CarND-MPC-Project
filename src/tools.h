#ifndef TOOLS_H
#define TOOLS_H

#include <tuple>
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

std::vector<double> eigen_to_std_vector(Eigen::VectorXd eigen) {
  auto begin = eigen.data();
  return std::vector<double>(begin, begin + eigen.size());
}

#endif /* TOOLS_H */
