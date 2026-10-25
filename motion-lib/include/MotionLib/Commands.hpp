#pragma once

#include "MotionLib/Drive.hpp"
#include <cmath>
#include <type_traits>

namespace motion {

// Sequential, blocking actions without per-line result checks. No threads,
// exceptions or allocation. A fatal failure latches off ALL later actions.
// Robot-specific hardware is accessed only through application callbacks.
class Commands {
public:
  explicit Commands(Drive& drive, void (*onFailure)() = nullptr,
                        void (*message)(const char*) = nullptr)
      : _drive(drive), _on_failure(onFailure), _message(message) {}

  Commands(const Commands&) = delete;
  Commands& operator=(const Commands&) = delete;

  // Configuration forwards to the underlying Drive. Feedback and failure
  // policy remain separate: Continue does not make distance moves open-loop.
  bool setMoveTolerance(double inches) { return _drive.setMoveTolerance(inches); }
  bool setStallTimeout(int milliseconds) { return _drive.setStallTimeout(milliseconds); }
  void setFailurePolicy(FailurePolicy policy) { _drive.setFailurePolicy(policy); }
  void setContinueCondition(bool (*condition)()) { _drive.setContinueCondition(condition); }

  bool setTurnTolerance(double degrees) {
    if (!std::isfinite(degrees) || degrees <= 0.0 || degrees > 180.0) return false;
    _turn_tolerance = degrees;
    return true;
  }
  // Applied to absolute turnTo targets only. Header start poses are already
  // in the selected field frame; relative turnBy angles are not transformed.
  void setHeadingTransform(double (*transform)(double)) { _heading_transform = transform; }

  void initPose(double x, double y, double heading) {
    if (!ready()) return;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(heading)) {
      fail("invalid starting pose");
      return;
    }
    _drive.init(x, y, heading);
  }
  void forward(double inches, double speed = 60.0) {
    if (ready()) accept(_drive.moveForward(inches, speed), "forward failed");
  }
  void backward(double inches, double speed = 60.0) {
    if (ready()) accept(_drive.moveBackward(inches, speed), "backward failed");
  }
  void forwardFor(double speed, int milliseconds) {
    if (ready()) accept(_drive.forwardFor(speed, milliseconds), "timed forward failed");
  }
  void backwardFor(double speed, int milliseconds) {
    if (ready()) accept(_drive.backwardFor(speed, milliseconds), "timed backward failed");
  }

  void turnTo(double heading, double speed = 25.0) {
    if (!ready()) return;
    if (_heading_transform) heading = _heading_transform(heading);
    _drive.odom().update();
    if (!std::isfinite(heading) || !std::isfinite(_drive.get_theta())) {
      fail("invalid turn pose");
      return;
    }
    checkTurn(_drive.turn(heading, speed), heading);
  }
  void turnBy(double degrees, double speed = 25.0) {
    if (!ready()) return;
    _drive.odom().update();
    const double target = _drive.get_theta() + degrees;
    if (!std::isfinite(target)) { fail("invalid relative turn"); return; }
    checkTurn(_drive.turnBy(degrees, speed), target, true);
  }

  void follow(const PathPoint* path, int length, double speed = 1.0,
              int stallMs = 0, int lastPoint = -1) {
    if (!ready()) return;
    const auto result = _drive.followPath(path, length, speed, lastPoint, stallMs);
    if (!ready()) return;
    if (!_drive.shouldContinue(result)) fail("path failed/cancelled");
  }

  // A plain robot function: action(clamp), action(startCascade, 500.0, 100.0).
  // void means the function completed; bool false aborts the sequence.
  // Long-running functions must implement their own cancellation checks.
  template <typename Function, typename... Args>
  void action(Function function, Args... args) {
    if (!ready()) return;
    if (missing(function, typename std::is_pointer<Function>::type())) {
      fail("null custom action");
      return;
    }
    typedef decltype(function(args...)) Result;
    static_assert(std::is_void<Result>::value || std::is_same<Result, bool>::value,
                  "Commands actions must return void or bool");
    invoke(function, typename std::is_void<Result>::type(), args...);
    ready(); // catch disable immediately after a custom function returns
  }

  void wait(int milliseconds) {
    if (!ready()) return;
    if (milliseconds < 0) { fail("invalid wait duration"); return; }
    vex::timer timer;
    timer.reset();
    while (ready() && timer.time(vex::msec) < milliseconds) {
      _drive.odom().update();
      vex::wait(fmax(0.0, fmin(10.0, milliseconds - timer.time(vex::msec))), vex::msec);
    }
  }

  void finish() {
    if (!ready()) return;
    _drive.stop();
    notify("Autonomous sequence complete");
    _active = false;
    _finished = true;
  }
  void abort() { if (_active) fail("sequence aborted"); }
  bool failed() const { return _failed; }
  bool finished() const { return _finished; }

private:
  Drive& _drive;
  void (*_on_failure)();
  void (*_message)(const char*);
  double (*_heading_transform)(double) = nullptr;
  double _turn_tolerance = 5.0;
  bool _active = true;
  bool _failed = false;
  bool _finished = false;

  void notify(const char* text) { if (_message) _message(text); }
  bool ready() {
    if (!_active) return false;
    if (!_drive.canContinue()) { fail("autonomous disabled"); return false; }
    return true;
  }
  void fail(const char* reason) {
    if (!_active) return;
    _active = false;
    _failed = true;
    _drive.stop();
    if (_on_failure) _on_failure(); // e.g. stop the robot's lift and toggle
    notify(reason);
  }
  void accept(MoveResult result, const char* reason) {
    if (!ready()) return;
    if (!_drive.shouldContinue(result)) { fail(reason); return; }
    if (result != MoveResult::Reached) notify("Motion stalled/timeout; continuing");
  }
  void checkTurn(MoveResult result, double target, bool relative = false) {
    if (!ready()) return;
    if (result == MoveResult::Invalid || result == MoveResult::Cancelled) {
      fail("turn invalid/cancelled");
      return;
    }
    _drive.odom().update();
    const double error = relative ? target - _drive.get_theta() :
                                   remainder(target - _drive.get_theta(), 360.0);
    // Retain the tested runner's heading guard even with Continue policy.
    if (!_drive.canContinue() || !std::isfinite(error) || fabs(error) > _turn_tolerance)
      fail("turn failed");
  }
  template <typename Function>
  static bool missing(Function function, std::true_type) { return function == nullptr; }
  template <typename Function>
  static bool missing(Function, std::false_type) { return false; }
  template <typename Function, typename... Args>
  void invoke(Function function, std::true_type, Args... args) { function(args...); }
  template <typename Function, typename... Args>
  void invoke(Function function, std::false_type, Args... args) {
    if (!function(args...)) fail("custom action failed");
  }
};

} // namespace motion
