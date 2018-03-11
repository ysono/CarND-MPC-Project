#ifndef MPC_H
#define MPC_H

#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return tuple with (
  //   optimal next steering actuation,
  //   optimal next acceleration actuation,
  //   x values of the optimal simulated trajectory,
  //   y values of the optimal simulated trajectory
  // )
  std::tuple<double, double, std::vector<double>, std::vector<double>>
  Solve(const std::vector<double> & init_state, const Eigen::VectorXd & coeffs);
};

#endif /* MPC_H */
