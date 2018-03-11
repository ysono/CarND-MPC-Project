#ifndef MPC_H
#define MPC_H

#include <list>
#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

extern const size_t solver_N;
extern const double solver_dt;

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
  Solve(const std::vector<double> & init_state, const Eigen::VectorXd & coeffs,
        const std::list<double> & steering_history, const std::list<double> & throttle_history);
};

#endif /* MPC_H */
