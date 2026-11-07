// MotionLib: Drive implementation.
// Ported from the proven autons.cpp - port order, timeouts, heading-hold
// factor, and settle logic are all UNCHANGED. Do not "clean up" the loops;
// every line here has a bench-test reason to exist.
//
// NOTE: no "using namespace vex" - our method named `forward` would collide
// with vex::forward (the direction enum value). Everything is qualified.
#include "MotionLib/Drive.hpp"
#include <math.h>
#include <cmath>

namespace motion {

static double wrap180(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

Drive::Drive(vex::motor_group& left, vex::motor_group& right,
             vex::rotation& pod_y, vex::rotation& pod_x, vex::inertial& imu,
             vex::brain& ui, double wheel_diam_in, double track_width_in,
             double y_offset_in, double x_offset_in)
    : _left(&left), _right(&right), _pod_y(&pod_y),
      _odom(pod_y, pod_x, imu, wheel_diam_in, y_offset_in, x_offset_in),
      _pursuit(_odom, left, right, ui, track_width_in),
      // STARTER VALUES (this robot's UN-optimized tune) - set your own via
      // setDriveGains/setTurnGains at app setup:
      _drive_pid(2.0, 0.001, 10.0, 3.0, 100.0, 0.5, 10),
      _turn_pid(1.8, 0.005, 12.0, 10.0, 100.0, 1.0, 10),
      _y_in_per_deg((M_PI * wheel_diam_in) / 360.0) {}

// ---------- tuning ----------
void Drive::setDriveGains(double kp, double ki, double kd, double start_i) {
  _drive_pid.set_constants(kp, ki, kd, start_i);
}

void Drive::setTurnGains(double kp, double ki, double kd, double start_i) {
  _turn_pid.set_constants(kp, ki, kd, start_i);
}

void Drive::setLookahead(double lookahead_in) {
  _pursuit.lookahead = lookahead_in;
}

void Drive::setSpeedProfile(double max_accel, double end_decel_zone,
                            double end_min_speed) {
  _pursuit.max_accel = max_accel;
  _pursuit.end_decel_zone = end_decel_zone;
  _pursuit.end_min_speed = end_min_speed;
}

// ---------- init ----------
void Drive::init(double start_x, double start_y, double start_heading) {
  _odom.reset(start_x, start_y, start_heading); // adopts current IMU direction
}

// ---------- drive (cruise + heading hold) ---------------------------------
// Distance comes from the Y tracking pod (true ground travel, ignores wheel
// slip). The IMU holds the starting heading so the robot drives straight.
void Drive::_drive(double distance, double speed) {
  double start = _pod_y->position(vex::degrees) * _y_in_per_deg;
  _drive_pid.max_output = speed;
  _drive_pid.set_target(start + distance);
  double hold = _odom.get_theta();

  vex::timer t;
  t.reset();
  int timeout = 2000 + (int)(fabs(distance) * 4000.0 / fmax(speed, 5.0)); // ms

  while (t.time(vex::msec) < timeout && !_drive_pid.is_settled()) {
    _odom.update();
    double power = _drive_pid.compute(_pod_y->position(vex::degrees) * _y_in_per_deg);
    double correction = wrap180(hold - _odom.get_theta()) * 0.8; // heading hold
    _left->spin(vex::forward, power + correction, vex::pct);
    _right->spin(vex::forward, power - correction, vex::pct);
    vex::wait(10, vex::msec);
  }

  _left->stop(vex::brake);
  _right->stop(vex::brake);
}

// ---------- turn ------------------------------------------------------------
// direction: 0 = shortest path, -1 = counterclockwise only, +1 = clockwise only
// Heading is continuous (odom), so constrained turns may travel past 360.
MoveResult Drive::_turn(double target_heading, double speed, int direction,
                        bool (*canContinue)(), bool relative) {
  const auto finish = [this](MoveResult result) { stop(); return result; };
  if (canContinue && !canContinue()) return finish(MoveResult::Cancelled);
  if (!std::isfinite(target_heading) || !std::isfinite(speed) || speed <= 0.0 || speed > 100.0)
    return finish(MoveResult::Invalid);
  _odom.update();
  double current = _odom.get_theta();
  if (!std::isfinite(current)) return finish(MoveResult::Invalid);
  double rel = fmod(target_heading - current, 360.0);
  double travel;
  if (relative) travel = target_heading;
  else if (direction == 0)     travel = wrap180(target_heading - current);
  else if (direction > 0) travel = (rel < 0.0) ? rel + 360.0 : rel; // cw:  0..360
  else                    travel = (rel > 0.0) ? rel - 360.0 : rel; // ccw: -360..0

  _turn_pid.max_output = speed;
  _turn_pid.set_target(current + travel);

  vex::timer t;
  t.reset();
  int timeout = 2000 + (int)(fabs(travel) * 400.0 / fmax(speed, 5.0)); // ms

  while (t.time(vex::msec) < timeout && !_turn_pid.is_settled()) {
    if (canContinue && !canContinue()) return finish(MoveResult::Cancelled);
    _odom.update();
    if (!std::isfinite(_odom.get_theta())) return finish(MoveResult::Invalid);
    double power = _turn_pid.compute(_odom.get_theta());
    _left->spin(vex::forward, power, vex::pct);
    _right->spin(vex::forward, -power, vex::pct);
    vex::wait(10, vex::msec);
  }

  return finish(_turn_pid.is_settled() ? MoveResult::Reached : MoveResult::Timeout);
}

// Coarse straight controller: preserve the robot-side tested control law.
MoveResult Drive::moveStraight(double inches, double max_speed,
                               bool (*canContinue)()) {
  const auto finish = [this](MoveResult result) {
    _left->stop(vex::brake);
    _right->stop(vex::brake);
    return result;
  };
  if (!canContinue) canContinue = _can_continue;
  if (canContinue && !canContinue()) return finish(MoveResult::Cancelled);
  if (!std::isfinite(inches) || !std::isfinite(max_speed) || max_speed <= 0.0 ||
      max_speed > 100.0 || !std::isfinite(_y_in_per_deg) || _y_in_per_deg <= 0.0)
    return finish(MoveResult::Invalid);
  _odom.update();
  const double start = _pod_y->position(vex::deg);
  const double heading = _odom.get_theta();
  const double direction = inches < 0.0 ? -1.0 : 1.0;
  const double target = fabs(inches);
  if (!std::isfinite(start) || !std::isfinite(heading))
    return finish(MoveResult::Invalid);
  vex::timer timer;
  timer.reset();
  double furthest = 0.0;
  double last_progress_ms = 0.0;
  while (!canContinue || canContinue()) {
    _odom.update();
    const double progress = direction * (_pod_y->position(vex::deg) - start) *
                            _y_in_per_deg;
    const double heading_error = remainder(heading - _odom.get_theta(), 360.0);
    if (!std::isfinite(progress) || !std::isfinite(heading_error))
      return finish(MoveResult::Invalid);
    const double remaining = target - progress;
    if (remaining <= _move_tolerance) {
      _odom.update();
      return finish(MoveResult::Reached);
    }
    if (progress > furthest + 0.25) {
      furthest = progress;
      last_progress_ms = timer.time(vex::msec);
    }
    if (_stall_timeout_ms > 0 && timer.time(vex::msec) - last_progress_ms >= _stall_timeout_ms)
      return finish(MoveResult::Stalled);
    if (timer.time(vex::msec) >= 1000 + target * 80.0)
      return finish(MoveResult::Timeout);
    const double power = direction * fmax(fmin(12.0, max_speed),
                                          fmin(3.8 * remaining, max_speed));
    const double correction = fmax(-15.0, fmin(0.8 * heading_error, 15.0));
    _left->spin(vex::forward,
                fmax(-max_speed, fmin(power + correction, max_speed)), vex::pct);
    _right->spin(vex::forward,
                 fmax(-max_speed, fmin(power - correction, max_speed)), vex::pct);
    vex::wait(10, vex::msec);
  }
  return finish(MoveResult::Cancelled);
}

bool Drive::arcade(double forward_input, double turn_input) {
  if (!std::isfinite(forward_input) || !std::isfinite(turn_input)) {
    stop();
    return false;
  }
  _odom.update();
  // Preserve the tested per-side saturation (not proportional normalization).
  _manual_left_volts = fmax(-12.0, fmin((forward_input + turn_input) / 8.3, 12.0));
  _manual_right_volts = fmax(-12.0, fmin((forward_input - turn_input) / 8.3, 12.0));
  _left->spin(vex::forward, _manual_left_volts, vex::volt);
  _right->spin(vex::forward, _manual_right_volts, vex::volt);
  return true;
}

void Drive::stop() {
  _manual_left_volts = _manual_right_volts = 0.0;
  _left->stop(vex::brake);
  _right->stop(vex::brake);
}

bool Drive::setMoveTolerance(double inches) {
  if (!std::isfinite(inches) || inches <= 0.0) return false;
  _move_tolerance = inches;
  return true;
}
bool Drive::setStallTimeout(int milliseconds) {
  if (milliseconds < 0) return false;
  _stall_timeout_ms = milliseconds;
  return true;
}
bool Drive::shouldContinue(MoveResult result) const {
  return result == MoveResult::Reached ||
      (_failure_policy == FailurePolicy::Continue &&
       (result == MoveResult::Stalled || result == MoveResult::Timeout));
}
bool Drive::shouldContinue(FollowResult result) const {
  return result == FollowResult::Reached ||
      (_failure_policy == FailurePolicy::Continue &&
       (result == FollowResult::Stalled || result == FollowResult::Timeout));
}
MoveResult Drive::moveForward(double inches, double speed) {
  if (!std::isfinite(inches) || inches < 0.0) { stop(); return MoveResult::Invalid; }
  return moveStraight(inches, speed);
}
MoveResult Drive::moveBackward(double inches, double speed) {
  if (!std::isfinite(inches) || inches < 0.0) { stop(); return MoveResult::Invalid; }
  return moveStraight(-inches, speed);
}
MoveResult Drive::_timed(double speed, int milliseconds, bool reverse) {
  const auto finish = [this](MoveResult result) { stop(); return result; };
  if (!std::isfinite(speed) || speed < 0.0 || speed > 100.0 || milliseconds < 0)
    return finish(MoveResult::Invalid);
  if (_can_continue && !_can_continue()) return finish(MoveResult::Cancelled);
  vex::timer timer;
  timer.reset();
  while (timer.time(vex::msec) < milliseconds) {
    if (_can_continue && !_can_continue()) return finish(MoveResult::Cancelled);
    _odom.update();
    if (!std::isfinite(_odom.get_x()) || !std::isfinite(_odom.get_y()) ||
        !std::isfinite(_odom.get_theta())) return finish(MoveResult::Invalid);
    const double power = reverse ? -speed : speed;
    _left->spin(vex::forward, power, vex::pct);
    _right->spin(vex::forward, power, vex::pct);
    // Avoid rounding a requested duration up by a whole 10-ms tick.
    vex::wait(fmin(10.0, milliseconds - timer.time(vex::msec)), vex::msec);
  }
  _odom.update();
  if (_can_continue && !_can_continue()) return finish(MoveResult::Cancelled);
  if (!std::isfinite(_odom.get_theta()) || !std::isfinite(_odom.get_x()) ||
      !std::isfinite(_odom.get_y())) return finish(MoveResult::Invalid);
  return finish(MoveResult::Reached); // requested duration elapsed, not distance
}
MoveResult Drive::forwardFor(double speed, int milliseconds) {
  return _timed(speed, milliseconds, false);
}
MoveResult Drive::backwardFor(double speed, int milliseconds) {
  return _timed(speed, milliseconds, true);
}
MoveResult Drive::turn(double heading, double speed) {
  if (std::isfinite(heading)) heading = remainder(heading, 360.0);
  return _turn(heading, speed, 0, _can_continue);
}
MoveResult Drive::turnBy(double degrees, double speed) {
  if (!std::isfinite(degrees) || fabs(degrees) > 360.0) { stop(); return MoveResult::Invalid; }
  return _turn(degrees, speed, 0, _can_continue, true);
}
FollowResult Drive::followPath(const PathPoint* pts, int len, double speed_scale,
                               int last_pt, int stall_timeout_ms) {
  return _pursuit.followResult(pts, len, speed_scale, last_pt,
                               stall_timeout_ms, _can_continue);
}

// ---------- public API ----------
void Drive::forward(double distance, double speed)  { _drive(distance, speed); }
void Drive::backward(double distance, double speed) { _drive(-distance, speed); }
void Drive::turnTo(double target_heading, double speed)    { _turn(target_heading, speed, 0); }
void Drive::turnLeft(double target_heading, double speed)  { _turn(target_heading, speed, -1); }
void Drive::turnRight(double target_heading, double speed) { _turn(target_heading, speed, 1); }

void Drive::follow(const PathPoint* pts, int len, double speed_scale,
                   int last_pt, int stall_timeout_ms) {
  _pursuit.follow(pts, len, speed_scale, last_pt, stall_timeout_ms);
}

FollowResult Drive::followSegment(const PathPoint* pts, int len,
                                  double speed_scale, PathDirection direction) {
  return _pursuit.followSegment(pts, len, speed_scale, direction);
}

} // namespace motion
