#include "MotionLib/Odom.hpp"

namespace motion {

Odom::Odom(vex::rotation& y_sensor, vex::rotation& x_sensor, vex::inertial& imu_sensor,
           double tracking_wheel_diam, double y_offset, double x_offset)
    : odom_y(&y_sensor), odom_x(&x_sensor), imu(&imu_sensor),
      wheel_diameter(tracking_wheel_diam), y_offset_in(y_offset),
      x_offset_in(x_offset) {
  in_per_deg = (M_PI * wheel_diameter) / 360.0;
}

void Odom::reset(double start_x, double start_y, double start_theta) {
  odom_y->resetPosition();
  odom_x->resetPosition();

  last_y_deg = 0.0;
  last_x_deg = 0.0;
  last_raw_deg = imu->heading(vex::degrees); // adopt current IMU direction
  heading_rad = start_theta * M_PI / 180.0;

  pose.x = start_x;
  pose.y = start_y;
  pose.theta = start_theta;
}

void Odom::update() {
  // --- continuous heading (path/field frame, never wraps) ------------------
  double raw = imu->heading(vex::degrees);
  double d = raw - last_raw_deg;
  while (d > 180.0) d -= 360.0;   // shortest-path delta: survives the
  while (d < -180.0) d += 360.0;  // 359.9 -> 0 crossing
  last_raw_deg = raw;

  double prev_heading_rad = heading_rad;
  heading_rad += d * M_PI / 180.0;
  double avg_heading = (prev_heading_rad + heading_rad) / 2.0;

  // --- tracking pod deltas --------------------------------------------------
  double current_y_deg = odom_y->position(vex::degrees);
  double current_x_deg = odom_x->position(vex::degrees);

  double delta_y_in = (current_y_deg - last_y_deg) * in_per_deg;
  double delta_x_in = (current_x_deg - last_x_deg) * in_per_deg;
  last_y_deg = current_y_deg;
  last_x_deg = current_x_deg;

  // --- lever-arm compensation (pods not on the centerlines) ----------------
  // Rotating by dtheta swings an off-center pod. Clockwise spin (dtheta > 0
  // here): a point s" to the robot's RIGHT moves BACKWARD (-s*dtheta phantom
  // forward travel on the Y pod), a point f" FORWARD of center moves RIGHT
  // (+f*dtheta phantom sideways travel on the X pod). Subtract the phantom
  // part so a spin in place integrates to zero translation. Offsets are 0
  // for centerline-mounted pods and these lines are no-ops.
  double dtheta_rad = heading_rad - prev_heading_rad; // clockwise-positive
  delta_y_in += y_offset_in * dtheta_rad;
  delta_x_in -= x_offset_in * dtheta_rad;

  // Local-to-global transformation
  pose.x += (delta_x_in * cos(avg_heading)) + (delta_y_in * sin(avg_heading));
  pose.y += (-delta_x_in * sin(avg_heading)) + (delta_y_in * cos(avg_heading));
  pose.theta = heading_rad * 180.0 / M_PI;
}

} // namespace motion
