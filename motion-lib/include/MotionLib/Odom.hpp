// MotionLib: 2-pod tracking-wheel odometry + inertial heading
// (VEX V5, bare metal).
// Ported verbatim from a proven bench implementation — the continuous-heading
// unwrap and local-to-global transform are the carefully-tuned parts; keep
// them as-is.
//
// UNITS: distances in INCHES (pass tracking_wheel_diam in inches; pose x/y in
// inches), angles in DEGREES. Consumers elsewhere are responsible for any
// other display unit.
#pragma once

#include "vex.h"
#include <math.h>

namespace motion {

struct Pose {
  double x = 0.0;     // inches
  double y = 0.0;     // inches
  double theta = 0.0; // degrees
};

class Odom {
public:
  vex::rotation* odom_y;
  vex::rotation* odom_x;
  vex::inertial* imu;

  double wheel_diameter; // inches
  double in_per_deg;

  // Pod placement (inches) relative to the rotation center, for lever-arm
  // compensation. y_offset: Y pod's distance to the RIGHT of the fore-aft
  // centerline (negative if left). x_offset: X pod's distance FORWARD of the
  // lateral centerline (negative if behind). 0 = pod centered (no
  // compensation). Wrong offsets make every rotation leak phantom travel
  // into the pose; verify by spinning 360° in place — pose must return to
  // its start. Sign axes match the pod readings (+delta_y = forward,
  // +delta_x = right).
  double y_offset_in = 0.0;
  double x_offset_in = 0.0;

  Pose pose;

  double last_y_deg = 0.0;
  double last_x_deg = 0.0;
  double last_raw_deg = 0.0;  // IMU raw (0-360) at last update
  double heading_rad = 0.0;   // continuous heading, path/field frame

  // y_offset_in / x_offset_in: tracking-pod lever arms (inches) — see the
  // member docs above. Leave at 0 for centerline-mounted pods.
  Odom(vex::rotation& y_sensor, vex::rotation& x_sensor, vex::inertial& imu_sensor,
       double tracking_wheel_diam, double y_offset_in = 0.0,
       double x_offset_in = 0.0);

  // reset(x, y, theta) declares the robot's CURRENT pose in the path/field
  // frame; the IMU's current reading is adopted as that heading. Calibrate
  // the IMU facing any direction - no need to align with the field.
  void reset(double start_x = 0.0, double start_y = 0.0, double start_theta = 0.0);
  void update();

  Pose get_pose() const { return pose; }
  double get_x() const { return pose.x; }
  double get_y() const { return pose.y; }
  double get_theta() const { return pose.theta; }
};

} // namespace motion
