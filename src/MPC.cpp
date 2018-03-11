#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using std::vector;
using CppAD::AD;

// The timestep length and duration
size_t N = 25;
double dt = 0.05;
// TODO reasons

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_v = 40; // TODO make it dynamic according to psi.

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;
size_t n_vars = a_start + N - 1;

size_t n_constraints = delta_start;

AD<double> polyeval_AD(const Eigen::VectorXd & coeffs, const AD<double> & x) {
  AD<double> result = 0.0;
  int sz = coeffs.size();
  for (int i = 0; i < sz; i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars) {
    
    // Express the cost, which is stored is the first element of `fg`.
    fg[0] = 0;
    for (unsigned int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    for (unsigned int t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }
    for (unsigned int t = 0; t < N - 2; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Express constraints
    //
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    //
    // The constrained expressions for the initial timestep. Keep these constant while solving.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    //
    // The constrained expressions for the future timesteps. Want to solve these expressions to be closer to zeros.
    for (unsigned int t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      // AD<double> cte0 = vars[cte_start + t - 1]; // not used
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> desired_y0 = polyeval_AD(coeffs, x0);
      AD<double> desired_psi0 = CppAD::atan(coeffs[1]);

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((desired_y0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - desired_psi0) + (v0 * delta0 / Lf * dt));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::tuple<double, double, vector<double>, vector<double>>
MPC::Solve(const vector<double> & init_state, const Eigen::VectorXd & coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double init_x = init_state[0];
  double init_y = init_state[1];
  double init_psi = init_state[2];
  double init_v = init_state[3];
  double init_cte = init_state[4];
  double init_epsi = init_state[5];

  // Initial value of the independent variables.
  // Set all values at future timesteps to zeros (set all to zeros for simplicity).
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set values at current timestep to init_state.
  vars[x_start] = init_x;
  vars[y_start] = init_y;
  vars[psi_start] = init_psi;
  vars[v_start] = init_v;
  vars[cte_start] = init_cte;
  vars[epsi_start] = init_epsi;

  // Lower and upper limits for the independent variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set no limits for measured state vars.
  for (unsigned int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // Limit steering to -25 and 25 degrees.
  for (unsigned int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // Limit acceleration to -1 and 1 m/s.
  for (unsigned int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints.
  //
  // We will constrain these expressions:
  // - state variables at initial timestep
  // - for future timesteps, diff between simulated value and actual value
  // And we want to limit the values of above expressions to be respectively:
  // - the values in init_state
  // - zeros
  // For all expressions, both lower and upper limits are set to the same value.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  // Set all to zeros for simplicity.
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = constraints_upperbound[i] = 0.0;
  }
  // Set constraints for the initial timestep
  constraints_lowerbound[x_start] = constraints_upperbound[x_start] = init_x;
  constraints_lowerbound[y_start] = constraints_upperbound[y_start] = init_y;
  constraints_lowerbound[psi_start] = constraints_upperbound[psi_start] = init_psi;
  constraints_lowerbound[v_start] = constraints_upperbound[v_start] = init_v;
  constraints_lowerbound[cte_start] = constraints_upperbound[cte_start] = init_cte;
  constraints_lowerbound[epsi_start] = constraints_upperbound[epsi_start] = init_epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (! ok) {
    std::cerr << "WARNING: solver was not successful" << std::endl;
  }

  // Cost
  // std::cout << "Cost " << solution.obj_value << std::endl;

  double next_delta = solution.x[delta_start];
  double next_a = solution.x[a_start];
  // For solved x and y, include the current timestep.
  vector<double> solved_x(N), solved_y(N);
  for (unsigned int i = 0; i < N; i++) {
    solved_x[i] = solution.x[x_start + i];
    solved_y[i] = solution.x[y_start + i];
  }

  return std::make_tuple(next_delta, next_a, solved_x, solved_y);
  // return solution.x;
  // return {solution.x[x_start + 1],   solution.x[y_start + 1],
  //         solution.x[psi_start + 1], solution.x[v_start + 1],
  //         solution.x[cte_start + 1], solution.x[epsi_start + 1],
  //         solution.x[delta_start],   solution.x[a_start]};
}
