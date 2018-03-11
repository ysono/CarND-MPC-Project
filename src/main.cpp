#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <list>
#include <thread>
#include <tuple>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "tools.h"

using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string & s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd & xvals, const Eigen::VectorXd & yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  int actuation_delay_ms = 100;
  double history_s = actuation_delay_ms / 1000.0;
  // Assert that the solver has enough time resolution.
  assert(history_s > solver_dt);
  // Assert that the solver's future outlook is long enough. Pad by an arbitrary factor.
  assert(solver_dt * solver_N > history_s * 5);
  size_t history_size = std::ceil(history_s / solver_dt);

  std::cout << "Remembering " << history_size << " past actuations" << std::endl;

  std::list<double> steering_history;
  std::list<double> throttle_history;
  for (unsigned int i = 0; i < history_size; i++) {
    // In the simulation, vehicle starts with 0 steering and 0 throttle.
    steering_history.push_back(0.0);
    throttle_history.push_back(0.0);
  }

  h.onMessage(
    [&mpc, &actuation_delay_ms, &steering_history, &throttle_history]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // transform the global coordinate to car's coordinate
          MatrixXd pts_wrt_car = translate_then_rotate(ptsx, ptsy, -px, -py, -psi);
          VectorXd ptsx_wrt_car = pts_wrt_car.row(0);
          VectorXd ptsy_wrt_car = pts_wrt_car.row(1);

          VectorXd coeffs = polyfit(ptsx_wrt_car, ptsy_wrt_car, 3);

          double cte = coeffs[0]; // no need for polyeval
          double epsi = -atan(coeffs[1]);

          // In the car's coordinate system, it is at origin pointing to x axis.
          vector<double> init_state{0, 0, 0, v, cte, epsi, 0};

          // Calculate steering angle and throttle using MPC.
          // Both are in between [-1, 1].
          //
          // If multithreading, multiple lambdas can update history.
          // - Ordering would not be guaranteed, but this is fine for use with solver
          //   as long as it's not wildly out of order
          // - Not sure how std::list is implemented, but traversing the lists
          //   could arrive at an erroneous `.end()` (seg fault), or could cause
          //   an infinite loop in pursuit of the `.end()` maybe? I'll worry about
          //   this if multithreading happens.
          //
          // Also, this assumes pinging of this server happens at ...
          // TODO timing
          double steering_value, throttle_value;
          vector<double> mpc_x, mpc_y;
          std::tie(steering_value, throttle_value, mpc_x, mpc_y) = mpc.Solve(
            init_state, coeffs,
            steering_history, throttle_history);

          steering_history.push_back(steering_value);
          steering_history.pop_front();
          throttle_history.push_back(throttle_value);
          throttle_history.pop_front();

          json msgJson;
          msgJson["steering_angle"] = -steering_value; // udacity simulator takes positive values for right turn
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory. Displayed in green line.
          msgJson["mpc_x"] = mpc_x;
          msgJson["mpc_y"] = mpc_y;

          //Display the waypoints/reference line.  Displayed in yellow line.
          msgJson["next_x"] = eigen_to_std_vector(ptsx_wrt_car);
          msgJson["next_y"] = eigen_to_std_vector(ptsy_wrt_car);

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(actuation_delay_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([/*&history_scheduler*/](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    // history_scheduler.detach();
  });

  h.onDisconnection([/*&history_scheduler*/](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    // history_scheduler.join();
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
