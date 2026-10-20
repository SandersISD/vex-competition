// MotionLib configuration: the distance-unit switch.
//
// This is the ONE library header that describes how YOUR app reads
// distances on screen (the motion API itself stays inches-only).
// Most apps never touch it - inches is the default.
#pragma once

// ------------------------------------------------------------------ units
// ONE distance unit for the whole app: path distances + brain-screen
// telemetry. Angles are always degrees. The LIBRARY is inches-only;
// convert ONLY here at the display (never inside PID/odometry).
//   #define UNIT_IN  -> inches
//   #define UNIT_MM  -> millimeters
//   #define UNIT_CM  -> centimeters
#define UNIT_IN

#if defined(UNIT_IN)
#define UNIT_NAME "in"
constexpr double UNIT_PER_IN = 1.0;               // inches per inch
#elif defined(UNIT_MM)
#define UNIT_NAME "mm"
constexpr double UNIT_PER_IN = 25.4;              // mm per inch
#else
#define UNIT_NAME "cm"
constexpr double UNIT_PER_IN = 2.54;              // cm per inch
#endif
