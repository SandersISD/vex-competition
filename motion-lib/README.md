# Simple autonomous API

For action lines without per-step result checks, see
[Commands](COMMANDS.md).

These calls keep controller loops out of the robot's autonomous sequence.
All distances are inches, angles are degrees and speeds are motor percentages.
Timed-move durations are milliseconds. All movement calls block until they
finish, cancel or time out; motor firmware can run mechanisms concurrently.

## Configure once

After constructing `drive`, configure the coarse straight controller:

```cpp
drive.setMoveTolerance(1.5);
drive.setStallTimeout(500);
drive.setFailurePolicy(motion::FailurePolicy::Stop);
drive.setContinueCondition(autonMayContinue);
```

`autonMayContinue` is an application function returning `bool`. It must
return promptly and should return false outside the permitted competition
mode. Clear it with `setContinueCondition(nullptr)` only if no mode check
is needed. This callback applies to the new APIs below, not old legacy calls.

The settings return `false` for an invalid tolerance or stall timeout without
changing the existing setting. Tolerance must be positive and finite. Stall
timeout must be nonnegative; zero disables the stall timer, not the total
movement timeout.

## Movement calls

```cpp
drive.init(start_x, start_y, start_heading);

auto result = drive.moveForward(25.0, 60.0);
if (!drive.shouldContinue(result)) return;

result = drive.moveBackward(16.0, 60.0);
if (!drive.shouldContinue(result)) return;

result = drive.turn(135.0, 25.0); // absolute, shortest arc
if (!drive.shouldContinue(result)) return;

result = drive.turnBy(-18.0, 25.0); // 18 degrees counterclockwise
if (!drive.shouldContinue(result)) return;

result = drive.forwardFor(75.0, 500); // timed seating push
if (!drive.shouldContinue(result)) return;
```

- `moveStraight(-17.0, 60.0)` is the signed-distance alternative.
- `moveForward` and `moveBackward` take nonnegative distances.
- `backwardFor(40.0, 200)` is the reverse timed move.
- `turnBy` accepts signed relative angles from -360 to +360 without resetting
  the IMU or odometry. `turn` uses an absolute heading and the shortest arc.
- `stop()` brakes only the drivetrain. Mechanism control belongs to the app.

The default coarse straight behavior matches the migrated robot controller:
P=3.8, 12% floor (limited by the requested speed), heading correction 0.8
capped at 15%, 1.5-inch acceptance, 500-ms no-progress timeout and total limit
of 1000 + 80 ms per inch. Acceptance occurs at braking, not after the robot
has physically settled. Very low speed limits may not overcome static friction.

Turns retain the existing turn PID gains, settling logic and timeout. There
is no 0.5-second turn cap. Timed moves do not enforce distance or heading;
Reached means the requested duration elapsed. All exits brake.

## Results and continuation

Straight, timed and turn methods return `motion::MoveResult`:
Reached, Stalled, Timeout, Invalid or Cancelled. Results report what happened;
setting a policy never changes a timeout into Reached.

`shouldContinue(result)` evaluates the configured policy:

- Stop (default): only Reached is accepted.
- Continue: Reached, Stalled and Timeout are accepted.
- Invalid and Cancelled are always rejected.

The policy does not execute the next action, stop mechanisms or latch future
calls off. Your route must check the result, as above. A Continue policy may
start the next action far from its intended pose. It does not provide obstacle
avoidance or confirm that a mechanism is safe to actuate.

## Paths

```cpp
auto result = drive.followPath(path, path_len, 0.8, -1, 700);
if (!drive.shouldContinue(result)) return;
```

`followPath` retains legacy last-positive-speed-point stopping but returns
`motion::FollowResult` and observes the configured continuation callback.
It validates input data and stops on non-finite odometry. The optional fifth
argument is a path-progress stall timer; zero disables it. It is separate
from the straight-movement stall setting.

FollowResult values are Reached, Stalled, Timeout, Invalid, Cancelled and
MissedEndpoint. Continue accepts only Reached/Stalled/Timeout; MissedEndpoint
is rejected. `followSegment` remains the advanced forward/reverse API and is
not automatically selected by `followPath`. It requires physical validation.

## Mechanisms stay robot-specific

Keep small functions in the robot project for cascade/claw wiring and polarity:

```cpp
startCascadeMove(500.0, 100.0); // nonblocking motor-position command
// Drive while the lift is moving.
waitForCascade(500.0);         // robot-specific wait/failure policy
releaseClamp();
```

These are application helpers, not MotionLib methods. Motor encoder degrees
and external mechanism-sensor degrees are not interchangeable.

## Compatibility

Existing `forward`, `backward`, `turnTo`, `turnLeft`, `turnRight` and `follow`
remain available. Coarse tolerance, stall policy and configured cancellation
apply to the new API only; legacy forward/backward retain their old PID
settling behavior. Do not assume the new failure policy controls legacy calls.
