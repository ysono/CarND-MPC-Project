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

using std::list;
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

enum actuation_delay_strategy {
  one,
  avg,
  iterative
};

int main(int argc, char* argv[]) {
  actuation_delay_strategy strategy;
  if (argc >= 2 && strcmp(argv[1], "avg") == 0) {
    strategy = avg;
  } else if (argc >= 2 && strcmp(argv[1], "iterative") == 0) {
    strategy = iterative;
  } else {
    strategy = one;
  }

  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  int actuation_delay_ms = 100;
  double actuation_delay_s = actuation_delay_ms / 1000.0;

  // In the simulation, vehicle starts with 0 steering and 0 throttle.
  double last_steering = 0;
  double last_throttle = 0;

  // List of tuples of (steering, throttle, timestamp).
  // Newer items will be pushed to the front, not back.
  list<std::tuple<double, double, std::time_t>> actuation_history =
    {std::make_tuple(last_steering, last_throttle, std::time(0))};

  h.onMessage(
    [&mpc, &actuation_delay_ms, &actuation_delay_s,
      &strategy,
      &last_steering, &last_throttle,
      &actuation_history]
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
          double psi = j[1]["psi"]; // radian
          double v = j[1]["speed"]; // mile/hour
          v /= mps_to_mph; // meter/sec

          // transform the global coordinate to car's coordinate system
          MatrixXd pts_wrt_car = translate_then_rotate(ptsx, ptsy, -px, -py, -psi);
          VectorXd ptsx_wrt_car = pts_wrt_car.row(0);
          VectorXd ptsy_wrt_car = pts_wrt_car.row(1);

          VectorXd coeffs = polyfit(ptsx_wrt_car, ptsy_wrt_car, 3);

          // Update and add state vars in the car's coordinate system
          px = py = psi = 0;
          double cte = coeffs[0];
          double epsi = -atan(coeffs[1]);

          // Now, determine the init state to pass to the solver.

          double aggregated_steering = 0; // used by `one` and `avg` strategies only
          double aggregated_throttle = 0; // ditto

          auto history_iter = actuation_history.begin(); // used by `avg` and `iterative` strategies only
          auto history_purge_iter = history_iter; // ditto

          std::time_t now = std::time(0);

          if (strategy == one) {
            aggregated_steering = last_steering;
            aggregated_throttle = last_throttle;
          } else {
            int actuation_i = 0;
            double aggregated_steering = 0;
            double aggregated_throttle = 0;

            // Determine the newest actuation that is older than the actuation delay.
            // If there is none older than the actuation delay, then choose the oldest in history.
            for(; history_iter != actuation_history.end(); history_iter++) {
              double steering, throttle;
              std::time_t ts;
              std::tie(steering, throttle, ts) = *history_iter;

              actuation_i++;
              aggregated_steering += steering;
              aggregated_throttle += throttle;

              double age = std::difftime(now, ts); // how long ago from the present this actuation was
              if (age > actuation_delay_s) {
                break;
              }
            }
            if (history_iter == actuation_history.end()) {
              // Business logic guarantees the list has at least one item, so this is safe.
              std::advance(history_iter, -1);
            }

            // save for purging, to be done later
            history_purge_iter = history_iter;

            if (strategy == avg) {
              aggregated_steering /= actuation_i;
              aggregated_throttle /= actuation_i;
            }
          }

          vector<double> init_state; // the init state to the pass to the solver.

          if (strategy == one || strategy == avg) {
            // helpers for the global kinetic model below. cos and sin are simplified away.
            double delayed_x_term = v /** cos(psi)*/ * actuation_delay_s;
            double delayed_y_term = 0; // v * sin(psi) * actuation_delay_s;
            double delayed_psi_term = v / Lf * aggregated_steering * actuation_delay_s;

            // global kinetic model for the actuation delay
            double px_delayed = px + delayed_x_term;
            double py_delayed = py + delayed_y_term;
            double psi_delayed = psi + delayed_psi_term;
            double v_delayed = v + aggregated_throttle * actuation_delay_s;
            double cte_delayed = cte + delayed_y_term;
            double epsi_delayed = epsi + delayed_psi_term;

            init_state = {px_delayed, py_delayed, psi_delayed, v_delayed, cte_delayed, epsi_delayed};
          } else {
            init_state = {px, py, psi, v, cte, epsi};

            // Iteratively update the states using global kinetic model to estimate
            // what the state will likely look like after actuation delay from the present.
            for(; history_iter != actuation_history.begin(); history_iter--) {
              double steering, throttle;
              std::time_t earlier_ts;
              std::tie(steering, throttle, earlier_ts) = *history_iter;

              double earlier_age = std::difftime(now, earlier_ts);
              earlier_age = std::min(earlier_age, actuation_delay_s); // cap by actuation delay

              double later_age;
              if (history_iter == actuation_history.begin()) {
                later_age = 0;
              } else {
                double _0, _1;
                std::time_t later_ts;
                std::tie(_0, _1, later_ts) = *(std::prev(history_iter, 1));
                later_age = std::difftime(now, later_ts);
              }

              double dt = earlier_age - later_age;

              init_state = global_kinetic_model(init_state, steering, throttle, dt, Lf);
            }
          }

          // Calculate steering angle and throttle using MPC.
          vector<double> mpc_x, mpc_y;
          std::tie(last_steering, last_throttle, mpc_x, mpc_y) = mpc.Solve(init_state, coeffs);

          json msgJson;
          msgJson["steering_angle"] = -last_steering; // udacity simulator takes positive values for right turn
          msgJson["throttle"] = last_throttle;

          //Display the MPC predicted trajectory. Displayed in green line.
          msgJson["mpc_x"] = mpc_x;
          msgJson["mpc_y"] = mpc_y;

          //Display the waypoints/reference line.  Displayed in yellow line.
          msgJson["next_x"] = eigen_to_std_vector(ptsx_wrt_car);
          msgJson["next_y"] = eigen_to_std_vector(ptsy_wrt_car);

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          // capture the time of actuation (just before the artificically introduced latency)
          now = std::time(0);

          auto response_thread = std::thread([&ws, &msg, &actuation_delay_ms]() {
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
          });

          if (strategy == avg || strategy == iterative) {
            // after actuation is executed, do cleanup
            // Here we push_back an item, keeping the size of the list at least one.
            actuation_history.push_front(std::make_tuple(last_steering, last_throttle, now));
            actuation_history.erase(history_purge_iter, actuation_history.end());
          }

          response_thread.join();
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

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
