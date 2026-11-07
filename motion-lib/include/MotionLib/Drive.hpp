// MotionLib: Drive - one object that IS the robot's motion system
// (VEX V5, bare metal).
//
// Beginner-friendly facade: the consumer hands over its own devices once,
// then drives with one-liners. No library file ever needs editing per robot.
//
// Included: PID cruise moves (heading-held), recorder-style turns,
// and pure-pursuit path following. Everything is BLOCKING (sequential
// auton code, exactly like the proven bench code it was ported from).
//
// UNITS: the LIBRARY is inches + degrees + motor % everywhere. The app's
// display unit (in/mm/cm) is one config switch: MotionLib/Units.hpp below.
#pragma once

#include "vex.h"
#include "MotionLib/Units.hpp"
#include "MotionLib/Pid.hpp"
#include "MotionLib/Odom.hpp"
#include "MotionLib/Pursuit.hpp"

namespace motion {

// Results describe what happened; the application decides whether to continue.
enum class MoveResult { Reached, Stalled, Timeout, Invalid, Cancelled };
enum class FailurePolicy { Stop, Continue };

class Drive {
public:
  // Hand over everything the robot IS - your own device objects, built in your
  // app's config. wheel_diam_in / track_width_in: MEASURE your robot (inches).
  // y_offset_in / x_offset_in: tracking-pod lever arms (inches) — Y pod's
  // distance RIGHT of the fore-aft centerline, X pod's distance FORWARD of
  // the lateral centerline (negative = left / behind). 0 = centered. See
  // Odom.hpp. ui: your brain; follow() telemetry prints there.
  Drive(vex::motor_group& left, vex::motor_group& right,
        vex::rotation& pod_y, vex::rotation& pod_x, vex::inertial& imu,
        vex::brain& ui, double wheel_diam_in, double track_width_in,
        double y_offset_in = 0.0, double x_offset_in = 0.0);

  // ------ tuning -----------------------------------------------------------
  // Starter gains (this robot's UN-optimized tune) are applied in the ctor;
  // replace them with your own at app setup. Kernel-agnostic: set once,
  // before autonomous, not per move.
  void setDriveGains(double kp, double ki, double kd, double start_i = 0.0);
  void setTurnGains(double kp, double ki, double kd, double start_i = 0.0);
  void setLookahead(double lookahead_in); // inches; bigger = smoother
  // Speed shaping for follow() (v0.1.1): max_accel = pct-per-second slew on
  // the speed command while ramping UP (decel is never limited);
  // end_decel_zone = inches before the last moving point where the command
  // tapers down (0 disables); end_min_speed = taper floor in pct so the
  // robot ARRIVES crawling instead of braking from cruise. Defaults:
  // 150, 8.0, 6.0. Call once at app setup, like the other tuning setters.
  void setSpeedProfile(double max_accel, double end_decel_zone = 8.0,
                       double end_min_speed = 6.0);

  // Declare the robot's CURRENT pose in the path/field frame. The IMU's
  // current reading is adopted as the heading, so calibrate the IMU facing
  // any direction. For a jerryio path: pass the path's first point and its
  // start heading (the _start_* constants of the generated header).
  void init(double start_x = 0.0, double start_y = 0.0, double start_heading = 0.0);

  // ------ recorder-style primitives (paste straight from your path notes) --
  // distance: inches | heading: absolute degrees (0-360) | speed: motor %
  // All blocking.
  void forward(double distance, double speed);
  void backward(double distance, double speed);
  void turnTo(double heading, double speed);    // shortest path
  void turnLeft(double heading, double speed);  // counterclockwise only
  void turnRight(double heading, double speed); // clockwise only

  // Coarse signed move, migrated from the tested robot controller.
  // +inches = forward, -inches = backward; accepts within 1.5 inches.
  // P=3.8; 12% minimum (or max_speed if lower); heading hold=0.8.
  // Stall: 500 ms without 0.25-inch progress; total: 1000+80 ms/inch.
  // No PID settling. Every exit brakes the drivetrain only.
  // Optional callback: false cancels (e.g. competition disabled).
  MoveResult moveStraight(double inches, double max_speed = 60.0,
                          bool (*canContinue)() = nullptr);

  // Friendly additive API. Legacy forward/backward/turnTo remain unchanged.
  bool setMoveTolerance(double inches); // finite, >0; default 1.5
  bool setStallTimeout(int milliseconds); // >=0; 0 disables; default 500
  void setFailurePolicy(FailurePolicy policy) { _failure_policy = policy; }
  void setContinueCondition(bool (*condition)()) { _can_continue = condition; }
  bool canContinue() const { return !_can_continue || _can_continue(); }
  bool shouldContinue(MoveResult result) const;
  bool shouldContinue(FollowResult result) const;
  void stop(); // drivetrain brake only

  MoveResult moveForward(double inches, double max_speed = 60.0);
  MoveResult moveBackward(double inches, double max_speed = 60.0);
  MoveResult forwardFor(double speed, int milliseconds);
  MoveResult backwardFor(double speed, int milliseconds);
  MoveResult turn(double heading, double speed = 25.0); // absolute, shortest arc
  MoveResult turnBy(double degrees, double speed = 25.0); // signed relative
  FollowResult followPath(const PathPoint* pts, int len, double speed_scale = 1.0,
                          int last_pt = -1, int stall_timeout_ms = 0);

  // Follow a generated path array (tools/path2h.py):
  //   drive.follow(path, path_len);
  // Blocking; ends within ~2" of the path's last moving point.
  // speed_scale: 1.0 = drawn speeds, 0.5 = half speed (bench testing).
  // last_pt (optional): index into pts[] to finish at once reached - use a
  // waypoint's pts_index (path_endpoints[i].pts_index in the generated
  // header) to stop there, do an action, then follow the next stretch.
  // Optional stall timer: ms without >=0.25-inch forward path progress.
  // 0 disables it; the existing total timeout is unchanged.
  void follow(const PathPoint* pts, int len, double speed_scale = 1.0,
              int last_pt = -1, int stall_timeout_ms = 0);

  // One endpoint-bounded slice; stops within 2 inches of its final sample.
  // Auto locks direction once. Returns failure instead of launching another leg.
  FollowResult followSegment(const PathPoint* pts, int len, double speed_scale,
                             PathDirection direction = PathDirection::Auto);

  // Nonblocking manual arcade: joystick inputs (-100..100), original /8.3
  // voltage mix, each side independently clamped to +/-12 V. Updates odometry.
  // Does NOT use the autonomous cancellation callback or failure latch.
  // Caller owns driver-mode gating. False on non-finite inputs (brakes).
  bool arcade(double forward_input, double turn_input);
  double get_left_command_volts() const { return _manual_left_volts; }
  double get_right_command_volts() const { return _manual_right_volts; }

  // ------ telemetry ----------------------------------------------------------
  // The same odometry the motion code uses - read it in your display loop.
  Odom& odom() { return _odom; }
  double get_x() const { return _odom.get_x(); }          // inches
  double get_y() const { return _odom.get_y(); }          // inches
  double get_theta() const { return _odom.get_theta(); }  // degrees
  Pose get_pose() const { return _odom.get_pose(); }

private:
  void _drive(double distance, double speed);
  MoveResult _turn(double target_heading, double speed, int direction,
                   bool (*canContinue)() = nullptr, bool relative = false);
  MoveResult _timed(double speed, int milliseconds, bool reverse);
  double _manual_left_volts = 0.0; // most recent arcade request, not measured voltage
  double _manual_right_volts = 0.0;
  double _move_tolerance = 1.5;
  int _stall_timeout_ms = 500;
  FailurePolicy _failure_policy = FailurePolicy::Stop;
  bool (*_can_continue)() = nullptr;

  vex::motor_group* _left;
  vex::motor_group* _right;
  vex::rotation* _pod_y; // the ground-truth distance wheel (cruise moves)

  Odom _odom;
  Pursuit _pursuit;
  Pid _drive_pid;
  Pid _turn_pid;
  double _y_in_per_deg; // inches per degree of the Y tracking wheel
};

} // namespace motion
