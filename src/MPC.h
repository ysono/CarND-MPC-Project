#ifndef MPC_H
#define MPC_H

#include <list>
#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

extern const double Lf;

const double mps_to_mph = 2.236936; // 1 meter/sec equals this much mile/hour

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
