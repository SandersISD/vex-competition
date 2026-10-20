# VEX Competition Robot

Snapshot of the current VEX V5 bare-metal C++ robot project. Uses the VEXcode
SDK and a vendored MotionLib copy. No earlier development history is included.

## Running

Open this folder with the VEX Robotics VS Code extension, select the V5 SDK,
build, and download. The project name is `vex-competition`; default slot is
retained in `.vscode/vex_project_settings.json`.

Competition autonomous and driver-control callbacks are enabled by default.
`BENCH_AUTON=true` in `src/main.cpp` instead runs autonomous once after a
controller A press/release; no competition driver callback runs in that mode.
A is not an emergency stop.

Start only with the cascade and side toggle at their known zero positions:
startup resets their encoder references. Place the robot so its initial
10-inch backward move ends at the path's declared start pose.

## Autonomous options

At the top of `src/main.cpp`:

- `START_TOGGLE_TWICE`: 1 commands 360 degrees reverse at startup; 0 commands
  180 degrees. The movement is nonblocking and continuous.
- `MIRROR_X_AXIS`: 1 selects the mirrored generated path header and mirrors
  handwritten absolute turn targets; 0 selects the original path.

The current snapshot preserves all source tuning and action values. Inspect
`autonomous()` before enabling the robot. Some timeout policies deliberately
continue to the next action without reaching the requested position or lift
height. The timed rush is not obstacle detection. Physical testing and clear
power-switch access are required; this snapshot is not a safety certification.

## Driver controls

- Axis3: forward/backward; Axis1: turn.
- R1/R2: cascade raise/lower; lowering uses the external zero check.
- Up: cascade motor position 0 plus claw command.
- B: claw command plus delayed cascade motor-position 100 preset.
- Down and X: opposite claw commands; verify pneumatic polarity physically.
- L1: side-toggle half-turn followed by the original sensor correction.

Driver controls use one mode-owned loop, not persistent broadcast tasks.

## Hardware

All motors use 18:1 gearing. Reversal flags are in `src/main.cpp`.

- Left drivetrain: ports 11, 1, 6.
- Right drivetrain: ports 12, 2, 7.
- Cascade motors: 9, 10; rotation sensor: 19.
- Side-toggle motors: 3, 4; rotation sensors: 13, 14.
- IMU: 18; Y/X tracking sensors: 15/16.
- Claw: Brain three-wire H.

## Files

- `src/main.cpp`: hardware, autonomous and driver control.
- `include/path_pin.h`: active original path data.
- `include/path_pin_mirrored.h`: mirrored alternative (include only one).
- `paths/`: editable JerryIO source exports, including the retained demo.
- `src/path-data*.md`: generated path references.
- `motion-lib/`: vendored library headers, sources and converter.

Regenerate the original or mirrored header from its corresponding source:

```sh
python3 motion-lib/tools/path2h.py paths/path_pin.txt include/path_pin.h
python3 motion-lib/tools/path2h.py paths/path_pin_mirrored.txt include/path_pin_mirrored.h
```

The converter also writes a companion reference; do not hand-edit generated
headers. Legacy `follow()` is forward-only and does not enforce final heading.
Reverse segment support is available but not used by this autonomous baseline.

Command-line build with your installed SDK path:

```sh
make T="/path/to/V5_20240802_15_00_00"
```

GNU-stack and RWX linker warnings occur with the bundled VEX toolchain.
