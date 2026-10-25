# Commands

`motion::Commands` runs readable action lines while handling failures
inside the library. No custom class, inheritance or robot-specific header is
needed. Keep the robot's hardware and mechanism functions in `main.cpp`.

## Example

After configuring a `motion::Drive` object, define ordinary robot functions:

```cpp
#include "MotionLib/Commands.hpp"

void clamp() { Claw.set(false); } // use this robot's actual polarity

void startCascade(double target, double speed) {
    Cascade.setVelocity(speed, vex::pct);
    Cascade.spinToPosition(target, vex::deg, false);
}

void autonomous() {
    motion::Commands robot(drive, stopAllMotors);

    robot.backward(16.0);
    robot.turnTo(135.0);
    robot.forward(25.0);
    robot.action(clamp);
    robot.action(startCascade, 500.0, 100.0);
    robot.wait(100);
    robot.forwardFor(75.0, 500);
    robot.finish();
}
```

`Claw`, `Cascade` and `stopAllMotors` are application-defined. Do not copy
mechanism polarity, gearing or limits from another robot without checking.

## Setup

Configure feedback and failure handling through Commands. Settings apply to
the underlying Drive object:

```cpp
motion::Commands robot(drive, stopAllMotors);
robot.setContinueCondition(autonMayContinue);
robot.setFailurePolicy(motion::FailurePolicy::Continue);
robot.setMoveTolerance(1.5);
robot.setStallTimeout(500);
```

Distance and heading commands still use sensor feedback under Continue policy.
Open-loop commands are explicit: `robot.forwardFor(75.0, 500)` requests motor
percentage for a duration; it does not target a distance or heading. Invalid
feedback and cancellation remain hard failures. The default Drive failure
policy is Stop; the tested competition app explicitly selects Continue.

Configure these once before running a sequence. Without a continuation
callback, the runner has no application-level competition-mode check.

Constructor arguments:

```cpp
motion::Commands robot(drive, stopAllMotors, reportMessage);
```

Only `drive` is required. `stopAllMotors` is an optional `void()` callback
invoked once on fatal failure. It should stop every mechanism the route may
have started. The runner always brakes the drivetrain itself. An optional
`void(const char*)` message callback can display failures and completion.
Callbacks must return promptly.

## Commands

- `initPose(x, y, heading)`: adopts the robot's current physical pose in field
  coordinates; do not use it to hide position errors mid-run.
- `forward(inches, speed=60)` / `backward(inches, speed=60)`: nonnegative
  distances, inches and motor percentage.
- `turnTo(heading, speed=25)`: absolute shortest-arc turn.
- `turnBy(degrees, speed=25)`: signed relative turn, -360 to +360 degrees.
- `forwardFor(speed, milliseconds)` / `backwardFor(speed, milliseconds)`:
  open-loop timed moves, with odometry updates and cancellation checks.
- `follow(path, length, speed=1, stallMs=0, lastPoint=-1)`: result-returning
  legacy path following. It is not automatic reverse-segment following.
- `wait(milliseconds)`: cancellable wait with odometry updates.
- `action(function, arguments...)`: guarded robot-specific function.
- `finish()`: brake drivetrain and permanently end this sequence instance.
- `abort()`: latch failure, brake and invoke the failure callback.
- `failed()` / `finished()`: inspect the final state; both false means no
  terminal state has been recorded yet.

After failure or finish, every later command is skipped. A new sequence
instance is required for a new run. Do not copy a sequence or use the same
instance concurrently from multiple tasks. The runner does not stop
background tasks started by custom actions; the failure callback must handle
those resources. An unfinished sequence is not automatically stopped by its
destructor; use `finish()` or `abort()` explicitly.

## Custom actions

Plain functions returning `void` or `bool` are supported, with or without
arguments. A bool result of false causes a fatal sequence failure. A null
function pointer is rejected. Other return types cause a compile-time error.

```cpp
robot.action(clamp);
robot.action(startCascade, 500.0, 100.0);
robot.action(waitForCascade, 500.0); // bool false stops all later actions
```

The runner checks cancellation before and after each custom function. It
cannot interrupt a blocking function internally; long functions must check
cancellation and enforce their own timeout. Nonblocking mechanism commands
return immediately and may continue while drivetrain actions execute.

## Turn policy and field transforms

Turns preserve normal MotionLib PID settling. Unlike straight/path moves,
turns always require final heading error within 5 degrees, even with Continue
policy. Set another positive finite tolerance up to 180 degrees using
`robot.setTurnTolerance(degrees)`. Invalid values return false without
changing the prior setting.

`robot.setHeadingTransform(fieldHeading)` optionally transforms handwritten
absolute `turnTo` targets. This does not transform path data, start poses or
relative `turnBy` angles. A mirrored path header already contains its own
transformed start pose; avoid mirroring it twice.

## Failure behavior

The Drive continuation policy accepts or rejects straight/timed/path results.
Invalid input, cancellation, a failed custom action or excessive turn error
always latches a fatal failure. The runner skips subsequent mechanism actions
as well as subsequent movement. Failure callbacks are called at most once.

Continue-after-timeout does not mean target arrival. A robot can still rush
or release a mechanism away from its intended position if that is the app's
chosen policy. This is not obstacle detection or a safety system.
