// MotionLib: PID controller (VEX V5, bare metal).
// Ported verbatim from a proven bench implementation — logic, comments, and
// gotchas preserved; class renamed + namespaced only. Unit-agnostic: the
// caller decides what "error" means (inches, degrees, ...).
#pragma once

#include <math.h>

namespace motion {

class Pid {
public:
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double start_i = 0.0;           // Error threshold below which integral accumulates
  double max_output = 100.0;      // Clamped output limit (e.g. 100% motor power)
  double settle_error = 1.0;      // Acceptable error considered "settled"
  int settle_cycles = 10;         // Consecutive cycles error must stay within
                                  // settle_error (e.g. 10 * 10ms = 100ms)

  double target = 0.0;
  double current = 0.0;
  double error = 0.0;
  double prev_error = 0.0;
  double integral = 0.0;
  double derivative = 0.0;
  double output = 0.0;

  int settled_count = 0;

  Pid() = default;
  Pid(double kp, double ki, double kd, double start_i = 0.0, double max_output = 100.0,
      double settle_error = 1.0, int settle_cycles = 10);

  void set_constants(double kp, double ki, double kd, double start_i = 0.0);
  void set_target(double target);
  double compute(double current_val);
  bool is_settled() const;
  void reset();
};

} // namespace motion
