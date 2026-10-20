#include "MotionLib/Pid.hpp"

namespace motion {

Pid::Pid(double kp, double ki, double kd, double start_i, double max_output,
         double settle_error, int settle_cycles)
    : kp(kp), ki(ki), kd(kd), start_i(start_i), max_output(max_output),
      settle_error(settle_error), settle_cycles(settle_cycles) {}

void Pid::set_constants(double p, double i, double d, double s_i) {
  kp = p;
  ki = i;
  kd = d;
  start_i = s_i;
}

void Pid::set_target(double t) {
  target = t;
  reset();
}

double Pid::compute(double current_val) {
  current = current_val;
  error = target - current;

  // Anti-windup: only accumulate integral near the target
  if (start_i != 0.0) {
    if (fabs(error) < start_i) {
      integral += error;
    } else {
      integral = 0.0;
    }
  } else {
    integral += error;
  }

  derivative = error - prev_error;
  prev_error = error;

  output = (kp * error) + (ki * integral) + (kd * derivative);

  // Clamp output
  if (output > max_output) output = max_output;
  if (output < -max_output) output = -max_output;

  // Settlement detection
  if (fabs(error) < settle_error) {
    settled_count++;
  } else {
    settled_count = 0;
  }

  return output;
}

bool Pid::is_settled() const {
  return settled_count >= settle_cycles;
}

void Pid::reset() {
  error = 0.0;
  prev_error = 0.0;
  integral = 0.0;
  derivative = 0.0;
  output = 0.0;
  settled_count = 0;
}

} // namespace motion
