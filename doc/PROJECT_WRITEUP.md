### The model

The WebSocket server receives from the simulator the measurements about the car's current position and movement as well as the car's pre-determined trajectory. The main function then converts the coordinates to the car's perspective, such that x axis points to the car's forward direction and y points to the left.

`MPC` class receives the measurements and sets them as the initial values for a state variable. This state is transformed over future timesteps using the global kinetic model. Actuation parameters of the global kinetic model at each timestep are optimized so that the resulting simulated future trajectory matches as closest as possible to the pre-determined trajectory.

The main function of the server receives the actuation values for the current timestep, whereas those for the remaining timesteps are discarded. The actuation is executed almost immediately, but with a delay of 100ms.

With the knowledge of this delay, the server always specifies to the solver that the first 100ms worth of actuaion values are not variable. The solver optimizes the future actuation beyond 100ms. From the solution, the earliest actuation beyond 100ms is executed.

### Choice of N and dt

The sample pair of N = 25 and dt = 0.05, provided in the MPC quiz, seems to have picked a sweet spot for the overall future duration for which to simulate and optimize global kinetmatic model iteration. `T = N * dt = 25 * 0.05 = 1.25 seconds`. If T was too long, the optimizer may have optimized distal timesteps at an excessive proportion, leaving the proximal timesteps less optimized. If T was too short, then the solution may not have prepared the car for the trajectory in the immediate future -- right position but wrong angle, for example.

The code has assertion that T be longer than 5 times the 100ms actuation delay. The factor of 5 was arbitrarily chosen. This is to ensure that the optimizer is given enough timesteps to optimize meaninfully. The code asserts that dt should not be larger than the 100ms actuation delay, because then the solver would become the bottleneck against improving the time resolution of actuation.

Having experimented with various T, it was reverted to the vicinity of 1.25 seconds. Next, N was reduced and dt was increased while keeping T about the same. This was for performance improvement. The resulting trajectory was qualitatively analyzed by how well it overall matched the pre-determined trajectory. The final choice was `N = 22, T = 0.06, T = 22 * 0.06 = 1.32 seconds`. Indeed it's basically the same as the original pair.

It seems that tuning T has a non-linear (subjectively speakign) effect on the solution. A slight increase in T might make the solution trajectory to curve wildly in the proximal end. Sometimes, penalizing large delta and/or large `d/dt delta` might fix it. It seemed much better strategy to tune the cost objective function rather than to tune N and dt.

### More thoughts

#### Mapping the trajectory

Since we are fitting the trajectory to a polynomial, the path needs to be unique for `x`, whatever dimension we choose as `x`. A hairpin turn would be difficult to fit. Since most trajectory travels forward, it makes sense to use the car's forward direction as x axis.

Also, the larger the epsi, the more cte differs from the actual closest distance between the trajectory and the car at the origin. For example, in an extreme case, if the trajectory is almost perpendicular to x but passes near the origin, the cte might register as a very large value, even though the actual distance between the origin and the line is small.

Maybe it might be better to fit a first-order polynomial to the trajectory and use it as x axis?

#### An attempt to manage curves more intelligently

I attempted to add centripetal acceleration as dependent variables. The aim was to use it to constrain longitudinal and lateral accelerations so that they were not simulatneously large in their absolute values.

`lateral_acceleration = V^2 / r`. To calculate the radius, I used the bicycle model (introduced at the beginning of this term with the kalman filter projects), and using `Lf` as the length; this is probably the wrong length.

![bicycle model](bicycle-model.png)

Also `Lf`'s unit must be `mile * sec / hour`, given how we calculate `psi` for the next timestep. This means values have to be converted between `meter / sec / sec` and `mile / hour / sec`.

In any case, I could not find a constraint that improved the solution trajectory. In fact, it made it either negligibly worse or much worse. There must be something wrong with my calculation. I left the lateral acceleration variables initialized to zeros and constrained to zeros, so hopefully ipopt is smart enough to eliminate unnecessary calculations.

Visualizing the vars and constraints was helpful. Please see `MPC::Solve()` for their visualization.

```
x0000 ... 00000
y0000 ... 00000
p0000 ... 00000
v0000 ... 00000
c0000 ... 00000
e0000 ... 00000
l0000 ... 00000
ddd00 ... 0000
aaa00 ... 0000
```

Instead of using lateral acceleration, curves can be similarly maanged if we automatically reduce speed when steering angle iss large. I left in this correlational cost:

```
fg[0] = some_multiplier *
  CppAD::pow(vars[delta_start + t] / max_delta, 2) *
  CppAD::pow(vars[v_start + t + 1] / optimal_speed_at_max_turn * relative_importance_of_speed, 2);
```

The final solution works fine at its target speed. It could go faster if speed limit is set higher. Because of various constraints, notably this correlational one, the actual speed does not really get close to the target speed limit.

#### Actuation delay

The car tends to stray outside the curve. This is because the period of simulation events is implicitly assumed to be the same as solver's `dt`. For a more comprehensive solution, we'd need a timer-based scheduler that periodically updates the history vectors with whatever last actuation values were: something like below, which I ended up taking out.

```
auto update_history = [&steering_history, &throttle_history, &last_steering_value, &last_throttle_value]() {
  // push first then pop, to avoid the unlikely case of starvation
  steering_history.push_back(last_steering_value);
  steering_history.pop_front();
  throttle_history.push_back(last_throttle_value);
  throttle_history.pop_front();
};
std::thread history_scheduler([&update_history, &history_ms]() {
  // Don't worry about thread safety of the updating of the histories. As long as the
  // historical values are approximately in order, it's good enough for the solver.
  //
  // Using read-committed isolation to read the last actuator values. With the
  // use of the simulation, there is only one thread writing to them. Updates
  // may happen in between scheduled executions, and those updates are ignored.
  std::thread(update_history).detach(); // Detach in order to keep clock more accurate. These threads are never joined.

  std::this_thread::sleep_for(std::chrono::milliseconds(history_ms));
});
```
