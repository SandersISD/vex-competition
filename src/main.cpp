#include "vex.h"
#include "MotionLib/Drive.hpp"
#include "MotionLib/Commands.hpp"
#include <math.h>
#include <cmath>


// Headers share symbol names; include exactly one to avoid double mirroring.
#define MIRROR_X_AXIS 0
#if MIRROR_X_AXIS
#include "path_pin_mirrored.h"
#else
#include "path_pin.h"
#endif

vex::brain Brain;
vex::controller Controller;
vex::competition Competition;
vex::digital_out Claw(Brain.ThreeWirePort.H);

vex::motor LeftMotorA(vex::PORT11, vex::ratio18_1, true);
vex::motor LeftMotorB(vex::PORT1, vex::ratio18_1, true);
vex::motor LeftMotorC(vex::PORT6, vex::ratio18_1, false);
vex::motor_group LeftDrive(LeftMotorA, LeftMotorB, LeftMotorC);

vex::motor RightMotorA(vex::PORT12, vex::ratio18_1, false);
vex::motor RightMotorB(vex::PORT2, vex::ratio18_1, false);
vex::motor RightMotorC(vex::PORT7, vex::ratio18_1, true);
vex::motor_group RightDrive(RightMotorA, RightMotorB, RightMotorC);

vex::inertial IMU(vex::PORT18);

vex::rotation OdomY(vex::PORT15, false); // forward/back wheel
vex::rotation OdomX(vex::PORT16, false); // strafe wheel

vex::motor PulleyMotorA(vex::PORT9, vex::ratio18_1, false);
vex::motor PulleyMotorB(vex::PORT10, vex::ratio18_1, true);
vex::motor_group PulleyDrive(PulleyMotorA, PulleyMotorB);
vex::rotation PulleyRotation(vex::PORT19, false);

vex::motor SideToggleL(vex::PORT3, vex::ratio18_1, true); // Left toggle motor (deg)
vex::motor SideToggleR(vex::PORT4, vex::ratio18_1, false); // Right toggle motor (deg)
vex::motor_group SideToggle(SideToggleL, SideToggleR);

vex::rotation LeftToggleRotation(vex::PORT13, true); // Left toggle rotation (deg)
vex::rotation RightToggleRotation(vex::PORT14, true);  // Right toggle rotation (deg)

constexpr double TRACKING_WHEEL_DIAMETER = 2.0;
constexpr double TRACK_WIDTH_INCHES = 13.15;
constexpr double Y_POD_OFFSET_IN  = 0.5;
constexpr double X_POD_OFFSET_IN  = -2.5;

motion::Drive drive(LeftDrive, RightDrive, OdomY, OdomX, IMU, Brain,
                    TRACKING_WHEEL_DIAMETER / UNIT_PER_IN, TRACK_WIDTH_INCHES,
                    Y_POD_OFFSET_IN, X_POD_OFFSET_IN);

// ------------------------------------------------------------------ custom actions functions
double toggleCounterL = 0.0;
double toggleCounterR = 0.0;

void flipToggle(double degrees) {
  const double targetL = LeftToggleRotation.position(vex::deg) - degrees;
  const double targetR = RightToggleRotation.position(vex::deg) + degrees;
  double errorL = targetL - LeftToggleRotation.position(vex::deg);
  double errorR = targetR - RightToggleRotation.position(vex::deg);

  while (fabs(errorL) > 1.0 || fabs(errorR) > 1.0) {
    SideToggleL.spin(vex::forward, errorL, vex::pct);
    SideToggleR.spin(vex::forward, -errorR, vex::pct);
    errorL = targetL - LeftToggleRotation.position(vex::deg);
    errorR = targetR - RightToggleRotation.position(vex::deg);
    vex::wait(10, vex::msec);
  }

}

void cascadeSpinTo(double target, double speed) {
  PulleyDrive.setStopping(vex::brake);
  PulleyDrive.setTimeout(2000, vex::msec);
  PulleyDrive.setVelocity(speed, vex::pct);
  PulleyDrive.spinToPosition(target, vex::deg, false);
}

void clamp() { Claw.set(false); }
void releaseClamp() { Claw.set(true); }

void stopAllMotors() {
  LeftDrive.stop(vex::brake);
  RightDrive.stop(vex::brake);
  PulleyDrive.stop(vex::brake);
  SideToggle.stop(vex::hold);
}

// ------------------------------------------------------------------ driver control
bool autonMayContinue() {
  return (Competition.isEnabled() && Competition.isAutonomous());
}

void init(){
    drive.setTurnGains(0.6, 0.0, 0.0, 0.0);
    drive.setLookahead(6.0);
    drive.setSpeedProfile(50.0, 8.0, 6.0);
    drive.setMoveTolerance(1.5);
    drive.setStallTimeout(500);
    drive.setFailurePolicy(motion::FailurePolicy::Continue);
    drive.setContinueCondition(autonMayContinue);
    
    clamp();
    Brain.Screen.print("Calibrating...");
    IMU.calibrate();
    while (IMU.isCalibrating()) vex::wait(20, vex::msec);
    
    PulleyRotation.resetPosition();
    PulleyDrive.resetPosition();
    LeftToggleRotation.resetPosition();
    RightToggleRotation.resetPosition();
    SideToggle.resetPosition();
}

// ---------------------------------------------------------------- autonomous
void autonomous() {
  motion::Commands robot(drive, stopAllMotors);
  robot.setTurnTolerance(5.0);

  // Prepare and approach the first pin.
  robot.action(flipToggle, 360.0);
  robot.action(clamp);
  robot.backward(10.0);

  robot.initPose(first_pin_start_x, first_pin_start_y, first_pin_start_h);
  robot.action(cascadeSpinTo, 500.0, 100.0); // raises while the robot follows the path
  robot.follow(first_pin, first_pin_len, 0.8, 700);
  robot.forwardFor(75.0, 500);

  // Drop the first pin.
  robot.action(cascadeSpinTo, 100.0, 100.0);
  robot.action(releaseClamp);
  robot.wait(100);

  // Approach the next set.
  robot.turnTo(180.0);
  robot.backward(16.0);
  robot.turnTo(135.0);
  robot.forward(25.0);

  robot.action(clamp);
  robot.action(cascadeSpinTo, 1000.0, 100.0);
  robot.wait(100);

  // Return to the base. These values preserve the current tested tuning.
  robot.backward(26.0);
  robot.turnTo(180.0);
  robot.forward(17.0);
  robot.forwardFor(75.0, 500);

  // Drop the next set.
  robot.action(cascadeSpinTo, 100.0, 100.0);
  robot.action(releaseClamp);

  robot.finish();
}

// ---------------------------------------------------------------- driver control

// Driver update state. Reset transient state when driver mode starts;
// toggleFault and toggleCounter intentionally persist across mode changes.
enum DriverCascadeState { CASCADE_MANUAL, CASCADE_B_DELAY, CASCADE_PRESET };
enum DriverToggleState { TOGGLE_IDLE, TOGGLE_HALF_TURN, TOGGLE_CORRECTION };
DriverCascadeState driverCascadeState = CASCADE_MANUAL;
DriverToggleState driverToggleState = TOGGLE_IDLE;
vex::timer cascadeTimer, toggleTimer, screenTimer, cascadeTick;
bool wasB = false, wasX = false, wasDown = false, wasL1 = false;
bool driverB = false;

void updateClawButtons() {
  const bool b = Controller.ButtonB.pressing();
  const bool x = Controller.ButtonX.pressing();
  const bool down = Controller.ButtonDown.pressing();
  if (down && !wasDown) Claw.set(true);
  if ((b && !wasB) || (x && !wasX)) Claw.set(false);
  wasB = b; wasX = x; wasDown = down;
}

void updateSideToggle() {
  const bool l1 = Controller.ButtonL1.pressing();
  if (l1 && !wasL1) {
    flipToggle(180.0);
  }
}

void driverControl() {
  PulleyDrive.setVelocity(100.0, vex::pct);
  SideToggle.setVelocity(100.0, vex::pct);
  SideToggle.setStopping(vex::hold);
  driverCascadeState = CASCADE_MANUAL;
  driverToggleState = TOGGLE_IDLE;
  cascadeTimer.reset();
  toggleTimer.reset();
  screenTimer.reset();
  cascadeTick.reset();
  wasB = Controller.ButtonB.pressing();
  wasX = Controller.ButtonX.pressing();
  wasDown = Controller.ButtonDown.pressing();
  wasL1 = Controller.ButtonL1.pressing();
  driverB = wasB;
  
  while (Competition.isEnabled() && Competition.isDriverControl()) {
    drive.arcade(Controller.Axis3.position(), Controller.Axis1.position());
    updateClawButtons();
    updateSideToggle();
    vex::wait(5, vex::msec);
  }
  stopAllMotors();
}

int main(){
    init();
    vex::competition::bStopTasksBetweenModes = true;
    Competition.drivercontrol(driverControl);
    Competition.autonomous(autonomous);
    Brain.Screen.printAt(10, 20, "Ready: competition driver/auton");

    while (true) {
        // Also stop mechanisms on disable; event handlers never own motors.
        if (!Competition.isEnabled()) stopAllMotors();
        vex::wait(20, vex::msec);
    }
}