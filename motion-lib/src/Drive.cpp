// MotionLib: Drive implementation.
// Ported from the proven autons.cpp - port order, timeouts, heading-hold
// factor, and settle logic are all UNCHANGED. Do not "clean up" the loops;
// every line here has a bench-test reason to exist.
//
// NOTE: no "using namespace vex" - our method named `forward` would collide
// with vex::forward (the direction enum value). Everything is qualified.
#include "MotionLib/Drive.hpp"
#include <math.h>

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
void Drive::_turn(double target_heading, double speed, int direction) {
  double current = _odom.get_theta();
  double rel = fmod(target_heading - current, 360.0);
  double travel;
  if (direction == 0)     travel = wrap180(target_heading - current);
  else if (direction > 0) travel = (rel < 0.0) ? rel + 360.0 : rel; // cw:  0..360
  else                    travel = (rel > 0.0) ? rel - 360.0 : rel; // ccw: -360..0

  _turn_pid.max_output = speed;
  _turn_pid.set_target(current + travel);

  vex::timer t;
  t.reset();
  int timeout = 2000 + (int)(fabs(travel) * 400.0 / fmax(speed, 5.0)); // ms

  while (t.time(vex::msec) < timeout && !_turn_pid.is_settled()) {
    _odom.update();
    double power = _turn_pid.compute(_odom.get_theta());
    _left->spin(vex::forward, power, vex::pct);
    _right->spin(vex::forward, -power, vex::pct);
    vex::wait(10, vex::msec);
  }

  _left->stop(vex::brake);
  _right->stop(vex::brake);
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
