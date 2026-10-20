#include "vex.h"
#include "MotionLib/Drive.hpp"
#include <math.h>
#include <cmath>

// Set to 1 to enable. Defaults preserve the current autonomous behavior.
#ifndef START_TOGGLE_TWICE
#define START_TOGGLE_TWICE 1
#endif
#ifndef MIRROR_X_AXIS
#define MIRROR_X_AXIS 0
#endif
#ifndef CLAMP_SECOND_PIN
#define CLAMP_SECOND_PIN 0
#endif

// Headers share symbol names; include exactly one to avoid double mirroring.
#if MIRROR_X_AXIS
#include "path_pin_mirrored.h"
#else
#include "path_pin.h"
#endif

// Only hand-written absolute turn targets need transforming at runtime.
double fieldHeading(double heading) {
  if (!MIRROR_X_AXIS) return heading;
  double mirrored = fmod(180.0 - heading, 360.0);
  return mirrored < 0.0 ? mirrored + 360.0 : mirrored;
}

// ------------------------------------------------------------------ hardware
vex::brain Brain;
vex::controller Controller;
vex::competition Competition;
vex::digital_out Solenoid(Brain.ThreeWirePort.H);

// Verified reversal flags: ports 11/1=true, port 6=false.
vex::motor LeftMotorA(vex::PORT11, vex::ratio18_1, true);
vex::motor LeftMotorB(vex::PORT1, vex::ratio18_1, true);
vex::motor LeftMotorC(vex::PORT6, vex::ratio18_1, false);
vex::motor_group LeftDrive(LeftMotorA, LeftMotorB, LeftMotorC);

// Verified reversal flags: ports 12/2=false, port 7=true.
vex::motor RightMotorA(vex::PORT12, vex::ratio18_1, false);
vex::motor RightMotorB(vex::PORT2, vex::ratio18_1, false);
vex::motor RightMotorC(vex::PORT7, vex::ratio18_1, true);
vex::motor_group RightDrive(RightMotorA, RightMotorB, RightMotorC);

// Inertial sensor: PORT18 verified on the robot.
vex::inertial IMU(vex::PORT18);

// Tracking pods: forward travel increases Y; rightward travel increases X.
vex::rotation OdomY(vex::PORT15, false); // forward/back wheel
vex::rotation OdomX(vex::PORT16, false); // strafe wheel

// Pulley hardware from the autonomous template.
// Cascade mapping from Downloads/LeftAuton.v5blocks (opposite old pulley flags).
vex::motor PulleyMotorA(vex::PORT9, vex::ratio18_1, false);
vex::motor PulleyMotorB(vex::PORT10, vex::ratio18_1, true);
vex::motor_group PulleyDrive(PulleyMotorA, PulleyMotorB);
vex::rotation PulleyRotation(vex::PORT19, false);

// Block-code names alias existing devices; do not construct duplicate devices.
vex::motor_group& Cascade = PulleyDrive;
vex::rotation& CascadeRotation = PulleyRotation; // block spelling: casacade
vex::digital_out& Claw = Solenoid;
vex::motor SideToggleA(vex::PORT3, vex::ratio18_1, true);
vex::motor SideToggleB(vex::PORT4, vex::ratio18_1, false);
vex::motor_group SideToggle(SideToggleA, SideToggleB);
vex::rotation LeftToggleRotation(vex::PORT13, true);
vex::rotation RightToggleRotation(vex::PORT14, true);


// ------------------------------------------------------------------ sizes
// Distance UNIT (UNIT_IN / UNIT_MM / UNIT_CM) is library config:
// MotionLib/Units.hpp (pulled in via Drive.hpp; default inches). Angles are
// always degrees. Convert ONLY at the boundaries: wheel size below
// (÷ UNIT_PER_IN) and the brain-screen display (× UNIT_PER_IN).

// Tracking wheel size, expressed in the chosen display unit (2.0 in)
constexpr double TRACKING_WHEEL_DIAMETER = 2.0;

// Distance between left/right wheel contact points (inches) - pure pursuit
constexpr double TRACK_WIDTH_INCHES = 13.15;

// Tracking-pod lever arms (inches) - lever-arm compensation in Odom. Y pod
// (vertical/forward) offset RIGHT of the fore-aft centerline = positive;
// X pod (horizontal/strafe) offset FORWARD of the lateral centerline =
// positive. This robot: Y pod 0.5" RIGHT of center, X pod 2.5" BEHIND center.
constexpr double Y_POD_OFFSET_IN  = 0.5;
constexpr double X_POD_OFFSET_IN  = -2.5;

// ------------------------------------------------------------ motion object
motion::Drive drive(LeftDrive, RightDrive, OdomY, OdomX, IMU, Brain,
                    TRACKING_WHEEL_DIAMETER / UNIT_PER_IN, TRACK_WIDTH_INCHES,
                    Y_POD_OFFSET_IN, X_POD_OFFSET_IN);

// ------------------------------------------------------------------ pulley
constexpr double PULLEY_MIN_LIMIT = -20.0;   // soft stops (deg)
constexpr double PULLEY_MAX_LIMIT = 700.0;
constexpr double PULLEY_KP        = 0.05;    // volts per degree of error
constexpr double PULLEY_DEADBAND  = 5.0;     // deg

double pulleyTarget = 0.0;

double pulleyPos() { return PulleyRotation.position(vex::deg); }

double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// One control step: drive toward pulleyTarget, or brake if close enough.
void pulleyHold() {
  double err = pulleyTarget - pulleyPos();
  if (fabs(err) < PULLEY_DEADBAND) {
    PulleyDrive.stop(vex::hold);
    return;
  }
  PulleyDrive.spin(vex::forward, clampd(PULLEY_KP * err, -12.0, 12.0), vex::volt);
}

// Blocking move for autonomous routes.
void pulleyGoTo(double target, int timeoutMs = 1500) {
  pulleyTarget = clampd(target, PULLEY_MIN_LIMIT, PULLEY_MAX_LIMIT);
  for (int t = 0; t < timeoutMs &&
                  fabs(pulleyTarget - pulleyPos()) > PULLEY_DEADBAND * 2; t += 10) {
    pulleyHold();
    vex::wait(10, vex::msec);
  }
  PulleyDrive.stop(vex::hold);
}

// ------------------------------------------------------------------ autonomous
constexpr int MECHANISM_TIMEOUT_MS = 2000;
constexpr bool BENCH_AUTON = false; // A-button one-shot test; false for competition
constexpr double END_HEADING_TOLERANCE_DEG = 5.0;
constexpr double CASCADE_POSITION_TOLERANCE_DEG = 15.0;

double toggleCounter = 0.0;
void stopAllMotors();

bool autonMayContinue() {
  return BENCH_AUTON || (Competition.isEnabled() && Competition.isAutonomous());
}

bool firstPinFailure(const char* reason) {
  stopAllMotors();
  Brain.Screen.printAt(10, 180, false, "FIRST PIN STOP: %s", reason);
  return false;
}

bool waitCascadeAt(double target) {
  vex::timer timer;
  timer.reset();
  while (autonMayContinue()) {
    drive.odom().update();
    const double a = PulleyMotorA.position(vex::deg);
    const double b = PulleyMotorB.position(vex::deg);
    if (!std::isfinite(a) || !std::isfinite(b))
      return firstPinFailure("invalid cascade position");
    if (fabs(target - a) <= CASCADE_POSITION_TOLERANCE_DEG &&
        fabs(target - b) <= CASCADE_POSITION_TOLERANCE_DEG) {
      Cascade.stop(vex::brake);
      return true;
    }
    if (timer.time(vex::msec) >= 500) {
      Cascade.stop(vex::brake);
      Brain.Screen.printAt(10, 180, "Cascade wait 0.5s; continuing");
      return true; // permission to proceed, NOT confirmation of target height
    }
    vex::wait(10, vex::msec);
  }
  return firstPinFailure("autonomous disabled");
}

bool checkedTurn(double heading) {
  if (!autonMayContinue()) return firstPinFailure("autonomous disabled");
  heading = fieldHeading(heading);
  drive.odom().update();
  const double initial = drive.get_theta();
  if (!std::isfinite(heading) || !std::isfinite(initial))
    return firstPinFailure("invalid turn pose");
  drive.turnTo(heading, 25.0);
  drive.odom().update();
  const double error = remainder(heading - drive.get_theta(), 360.0);
  if (!autonMayContinue() || !std::isfinite(error) ||
      fabs(error) > END_HEADING_TOLERANCE_DEG)
    return firstPinFailure("turn failed");
  return true;
}

// Coarse signed-distance move: no PID settling/repeated endpoint correction.
// Stop on first entry within 1.5 inches. Max 60%, heading hold as in Drive.
// A true return permits continuation; it does not guarantee target arrival.
// By request, 0.5s without progress brakes then continues by default.
bool checkedStraight(double inches, bool continueOnStall = true) {
  if (!autonMayContinue()) return firstPinFailure("autonomous disabled");
  if (!std::isfinite(inches)) return firstPinFailure("invalid distance");
  drive.odom().update();
  const double start = OdomY.position(vex::deg);
  const double heading = drive.get_theta();
  const double direction = inches < 0.0 ? -1.0 : 1.0;
  const double target = fabs(inches);
  const double inPerDegree = M_PI * (TRACKING_WHEEL_DIAMETER / UNIT_PER_IN) / 360.0;
  if (!std::isfinite(start) || !std::isfinite(heading))
    return firstPinFailure("invalid straight pose");
  vex::timer timer;
  timer.reset();
  double furthest = 0.0;
  double lastProgressMs = 0.0;
  while (autonMayContinue()) {
    drive.odom().update();
    const double progress = direction * (OdomY.position(vex::deg) - start) * inPerDegree;
    const double headingError = remainder(heading - drive.get_theta(), 360.0);
    if (!std::isfinite(progress) || !std::isfinite(headingError))
      return firstPinFailure("invalid straight feedback");
    const double remaining = target - progress;
    if (remaining <= 1.5) {
      LeftDrive.stop(vex::brake);
      RightDrive.stop(vex::brake);
      drive.odom().update();
      // Acceptance is at brake command; physical braking can add travel.
      return true;
    }
    if (progress > furthest + 0.25) {
      furthest = progress;
      lastProgressMs = timer.time(vex::msec);
    }
    if (timer.time(vex::msec) - lastProgressMs >= 500 ||
        timer.time(vex::msec) >= 1000 + target * 80.0) {
      if (!continueOnStall) return firstPinFailure("straight stalled/timeout");
      // Brake the drivetrain only: do not interrupt the concurrent lift.
      LeftDrive.stop(vex::brake);
      RightDrive.stop(vex::brake);
      Brain.Screen.printAt(10, 180, "Straight stalled/timeout; continuing");
      return true;
    }
    const double power = direction * clampd(3.8 * remaining, 12.0, 60.0);
    const double correction = clampd(0.8 * headingError, -15.0, 15.0);
    LeftDrive.spin(vex::forward, clampd(power + correction, -60.0, 60.0), vex::pct);
    RightDrive.spin(vex::forward, clampd(power - correction, -60.0, 60.0), vex::pct);
    vex::wait(10, vex::msec);
  }
  return firstPinFailure("autonomous disabled");
}

// Shared physical-base seating push. This is not obstacle detection.
bool rushForward() {
  if (!autonMayContinue()) return firstPinFailure("autonomous disabled");
  vex::timer alignment;
  alignment.reset();
  while (alignment.time(vex::msec) < 500 && autonMayContinue()) {
    LeftDrive.spin(vex::forward, 75.0, vex::pct);
    RightDrive.spin(vex::forward, 75.0, vex::pct);
    drive.odom().update();
    vex::wait(10, vex::msec);
  }
  LeftDrive.stop(vex::brake);
  RightDrive.stop(vex::brake);
  if (!autonMayContinue()) return firstPinFailure("autonomous disabled");
  return true;
}

void autonomous() {
  if (!autonMayContinue()) return;
  SideToggle.setVelocity(100.0, vex::pct);
  SideToggle.setStopping(vex::hold);
  SideToggle.setTimeout(2000, vex::msec);
  SideToggle.spinFor(vex::reverse, START_TOGGLE_TWICE ? 360.0 : 180.0,
                     vex::deg, false);
  toggleCounter += START_TOGGLE_TWICE ? 360.0 : 180.0;
  // Startup already initialized odometry. Back into the drawn starting pose
  // before adopting the field coordinates; do not move the robot afterward.
  Claw.set(false);
  if (!checkedStraight(-10.0)) return;


  drive.init(first_pin_start_x, first_pin_start_y, first_pin_start_h);
  Cascade.setStopping(vex::brake);
  Cascade.setTimeout(MECHANISM_TIMEOUT_MS, vex::msec);
  Cascade.setVelocity(100.0, vex::pct);
  Cascade.spinToPosition(500.0, vex::deg, false);

  // Stop pursuit after 700 ms without forward path progress, not total duration.
  // Intentionally continue to the turn/rush even if pursuit stalls/times out.
  // Geometry and start pose are already mirrored in the selected header.
  drive.follow(first_pin, first_pin_len, 0.8, -1, 700);
  if (!autonMayContinue()) { firstPinFailure("autonomous disabled"); return; }

  // Do not require endpoint position after a stalled approach. Keep invalid
  // odometry, disabled-mode, lift and final-heading checks as hard failures.

  // drive.odom().update();
  // if (!std::isfinite(drive.get_x()) || !std::isfinite(drive.get_y()) ||
  //     !std::isfinite(drive.get_theta())) {
  //   firstPinFailure("invalid approach pose");
  //   return;
  // }
  // if (!waitCascadeAt(500.0)) return;

  // const double endHeading = first_pin_endpoints[first_pin_endpoints_len - 1].heading;
  // if (!std::isfinite(endHeading) || endHeading < 0.0) {
  //   firstPinFailure("invalid endpoint heading");
  //   return;
  // }

  // if (!checkedTurn(fieldHeading(endHeading))) return; // header heading already mirrored

  if (!rushForward()) return;

  // Lower to an absolute MOTOR position only AFTER seating stops.
  Cascade.setVelocity(80.0, vex::pct);
  Cascade.spinToPosition(100.0, vex::deg, false);
  if (!waitCascadeAt(100.0)) return;
  Cascade.setVelocity(100.0, vex::pct);
  Claw.set(true);
  vex::wait(100, vex::msec); // original pause after the first-pin claw action

  if (!checkedTurn(180.0)) return;
  if (!checkedStraight(-16.0)) return;
  if (!checkedTurn(135.0)) return;
  if (!checkedStraight(25.0)) return;
  if (!autonMayContinue()) { firstPinFailure("autonomous disabled"); return; }

  #if(CLAMP_SECOND_PIN)

  Claw.set(false); // reclamp the next set
  Cascade.setVelocity(80.0, vex::pct);
  Cascade.spinToPosition(1000.0, vex::deg, false);
  vex::wait(100, vex::msec); // allow the pneumatic action before retreating

  if (!checkedStraight(-26.0)) return;
  if (!checkedTurn(180.0)) return;
  if (!checkedStraight(17.0, true)) return; // may seat against the base early
  if (!waitCascadeAt(500.0)) return;
  if (!rushForward()) return;

  // Return rush has stopped: lower, verify, then release the next set.
  Cascade.setVelocity(80.0, vex::pct);
  Cascade.spinToPosition(100.0, vex::deg, false);
  if (!waitCascadeAt(100.0)) return;
  Cascade.setVelocity(100.0, vex::pct);
  if (!autonMayContinue()) { firstPinFailure("autonomous disabled"); return; }
  Claw.set(true);

  #endif

  LeftDrive.stop(vex::brake);
  RightDrive.stop(vex::brake);
  Brain.Screen.printAt(10, 180, "Second set dropped; stopped");

  
}

// ------------------------------------------------------------------ driver
// Preserve block-code voltage mixing, presets and toggle correction formula.
// A single mode-owned loop replaces permanent event/broadcast workers.
bool toggleFault = false;

void stopAllMotors() {
  LeftDrive.stop(vex::brake);
  RightDrive.stop(vex::brake);
  Cascade.stop(vex::brake);
  SideToggle.stop(vex::hold);
}

void driverControl() {
  // Do not reset IMU/pods on mode changes: preserve MotionLib's pose.
  // Mechanism zeroing happens once in main, at a known physical home pose.
  Cascade.setVelocity(100.0, vex::pct);
  SideToggle.setVelocity(100.0, vex::pct);
  SideToggle.setStopping(vex::hold);
  enum CascadeState { MANUAL, B_DELAY, PRESET };
  CascadeState cascadeState = MANUAL;
  enum ToggleState { IDLE, HALF_TURN, CORRECTION };
  ToggleState toggleState = IDLE;
  vex::timer cascadeTimer, toggleTimer, screenTimer, cascadeTick;
  bool wasB = Controller.ButtonB.pressing();
  bool wasX = Controller.ButtonX.pressing();
  bool wasDown = Controller.ButtonDown.pressing();
  bool wasL1 = Controller.ButtonL1.pressing();
  double leftV = 0.0, rightV = 0.0;

  while (Competition.isEnabled() && Competition.isDriverControl()) {
    drive.odom().update();
    leftV = clampd((Controller.Axis3.position() + Controller.Axis1.position()) / 8.3,
                   -12.0, 12.0);
    rightV = clampd((Controller.Axis3.position() - Controller.Axis1.position()) / 8.3,
                    -12.0, 12.0);
    LeftDrive.spin(vex::forward, leftV, vex::volt);
    RightDrive.spin(vex::forward, rightV, vex::volt);

    const bool b = Controller.ButtonB.pressing();
    const bool x = Controller.ButtonX.pressing();
    const bool down = Controller.ButtonDown.pressing();
    const bool l1 = Controller.ButtonL1.pressing();
    if (down && !wasDown) Claw.set(true);
    if ((b && !wasB) || (x && !wasX)) Claw.set(false);
    wasB = b; wasX = x; wasDown = down;

    // Equivalent to the block's ~100-ms cascade polling. Preset moves are
    // nonblocking here so joystick control continues while the lift moves.
    if (cascadeTick.time(vex::msec) >= 100) {
      cascadeTick.reset();
      if (cascadeState == PRESET) {
        if (Cascade.isDone() || cascadeTimer.time(vex::msec) >= MECHANISM_TIMEOUT_MS) {
          if (!Cascade.isDone())
            Brain.Screen.printAt(10, 140, "Cascade preset TIMEOUT");
          Cascade.setStopping(vex::brake);
          Cascade.stop();
          cascadeState = MANUAL;
        }
      } else if (cascadeState == B_DELAY) {
        if (cascadeTimer.time(vex::msec) >= 200) {
          Cascade.spinToPosition(100.0, vex::deg, false);
          cascadeTimer.reset();
          cascadeState = PRESET;
        }
      } else if (Controller.ButtonUp.pressing()) {
        Claw.set(true);
        Cascade.spinToPosition(0.0, vex::deg, false);
        cascadeTimer.reset();
        cascadeState = PRESET;
      } else if (b) {
        Cascade.stop();
        cascadeTimer.reset();
        cascadeState = B_DELAY;
      } else if (Controller.ButtonR1.pressing()) {
        Cascade.spin(vex::forward);
      } else if (Controller.ButtonR2.pressing()) {
        if (CascadeRotation.position(vex::deg) < 0.0) Cascade.stop();
        else Cascade.spin(vex::reverse);
      } else {
        Cascade.stop();
      }
    }

    if (l1 && !wasL1 && toggleState == IDLE && !toggleFault) {
      toggleCounter += 180.0;
      SideToggle.spinFor(vex::reverse, 180.0, vex::deg, false);
      toggleTimer.reset();
      toggleState = HALF_TURN;
    }
    wasL1 = l1;
    if (toggleState != IDLE) {
      if (toggleTimer.time(vex::msec) >= MECHANISM_TIMEOUT_MS) {
        SideToggle.stop(vex::hold);
        toggleFault = true; // unknown position: do not compound another half-turn
        toggleState = IDLE;
      } else if (SideToggle.isDone()) {
        if (toggleState == HALF_TURN) {
          const double error = toggleCounter +
              (LeftToggleRotation.position(vex::deg) -
               RightToggleRotation.position(vex::deg)) / 2.0;
          if (fabs(error) > 1.0) {
            SideToggle.spinFor(error > 0 ? vex::reverse : vex::forward,
                               fabs(error), vex::deg, false);
            toggleTimer.reset();
            toggleState = CORRECTION;
          } else toggleState = IDLE;
        } else toggleState = IDLE;
      }
    }

    if (screenTimer.time(vex::msec) >= 200) {
      screenTimer.reset();
      Brain.Screen.printAt(10, 20, "Cascade:%8.1f", CascadeRotation.position(vex::deg));
      Brain.Screen.printAt(10, 50, "Left V:%6.2f Right V:%6.2f", leftV, rightV);
      Brain.Screen.printAt(10, 80, "X:%7.1f Y:%7.1f H:%7.1f",
                           drive.get_x(), drive.get_y(), drive.get_theta());
      if (toggleFault) Brain.Screen.printAt(10, 110, "Toggle TIMEOUT: restart at home");
    }
    vex::wait(5, vex::msec);
  }
  stopAllMotors();
}

// ------------------------------------------------------------------ main
int main() {
  drive.setTurnGains(0.6, 0.0, 0.0, 0.0);
  drive.setLookahead(6.0);
  drive.setSpeedProfile(50.0, 8.0, 6.0);

  Solenoid.set(false);
  Brain.Screen.print("Calibrating...");
  IMU.calibrate();
  while (IMU.isCalibrating()) vex::wait(20, vex::msec);

  // Start only with cascade and toggle physically at their known zero.
  CascadeRotation.resetPosition();
  Cascade.resetPosition();
  LeftToggleRotation.resetPosition();
  RightToggleRotation.resetPosition();
  SideToggle.resetPosition();
  drive.init();
  Brain.Screen.clearScreen();

  if (BENCH_AUTON) {
    Brain.Screen.printAt(10, 20, "BENCH: place robot, press/release A");
    while (!Controller.ButtonA.pressing()) vex::wait(20, vex::msec);
    while (Controller.ButtonA.pressing()) vex::wait(20, vex::msec);
    autonomous();
    stopAllMotors();
    while (true) vex::wait(100, vex::msec);
  }

  vex::competition::bStopTasksBetweenModes = true;
  Competition.autonomous(autonomous);
  Competition.drivercontrol(driverControl);
  Brain.Screen.printAt(10, 20, "Ready: competition driver/auton");
  while (true) {
    // Also stop mechanisms on disable; event handlers never own motors.
    if (!Competition.isEnabled()) stopAllMotors();
    vex::wait(20, vex::msec);
  }
}
