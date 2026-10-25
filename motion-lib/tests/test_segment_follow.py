"""Host-side segment controller tests with a small differential-drive model.
Run: python3 tests/test_segment_follow.py. Requires clang++ or g++.
This checks logic and command signs, not physical tracking accuracy.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]

VEX = r"""
#pragma once
#include <cmath>
namespace vex {
inline double now = 0, left = 0, right = 0;
enum units { msec, sec, forward, pct, brake };
inline void wait(double t, units) { now += t; }
struct motor_group {
  bool is_left; double first = 0;
  void spin(units, double v, units) {
    if (first == 0 && v != 0) first = v;
    (is_left ? left : right) = v;
  }
  void stop(units) { (is_left ? left : right) = 0; }
};
struct brain { struct screen {
  template<class... T> void printAt(int, int, const char*, T...) {}
} Screen; };
struct timer {
  double start = now;
  void reset() { start = now; }
  double time(units u) { return (now-start)/(u == sec ? 1000 : 1); }
};
}
"""
ODOM = r"""
#pragma once
namespace motion {
struct Odom {
  double x=0, y=0, heading=180, previous=0; bool frozen=false;
  void update() {
    double dt=(vex::now-previous)/1000; previous=vex::now;
    if (frozen) return;
    double turn=(vex::left-vex::right)*0.6/13.15;
    double mid=heading*M_PI/180 + turn*dt/2;
    double travel=(vex::left+vex::right)*0.3*dt;
    x+=travel*sin(mid); y+=travel*cos(mid);
    heading+=turn*dt*180/M_PI;
  }
  double get_x() { return x; }
  double get_y() { return y; }
  double get_theta() { return heading; }
};
}
"""
TEST = r"""
#include "MotionLib/Pursuit.hpp"
#include <cassert>
#include <limits>
int main() {
  using namespace motion;
  Odom o; vex::motor_group l{true}, r{false}; vex::brain b;
  Pursuit f(o,l,r,b,13.15,6);
  PathPoint back[]={{0,0,30},{0,4,30},{0,8,0}};
  assert(f.followSegment(back,3,0.3)==FollowResult::Reached);
  assert(l.first<0 && r.first<0); // Auto selected reverse at heading 180
  assert(o.y>6 && o.y<10); // zero-speed endpoint, not last moving sample
  assert(vex::left==0 && vex::right==0); // brake at the reversal

  l.first=r.first=0;
  PathPoint forward[]={{0,8,30},{0,4,30},{0,0,0}};
  assert(f.followSegment(forward,3,0.3)==FollowResult::Reached);
  assert(l.first>0 && r.first>0); // same actual heading, now forward

  o.x=o.y=0; o.heading=90;
  assert(f.followSegment(back,3,0.3)==FollowResult::Invalid); // ambiguous Auto
  o.heading=180; o.frozen=true;
  assert(f.followSegment(back,3,0.3)==FollowResult::Timeout);
  assert(vex::left==0 && vex::right==0);
  assert(f.followSegment(nullptr,3,0.3)==FollowResult::Invalid);
  assert(f.followSegment(back,1,0.3)==FollowResult::Invalid);
  assert(f.followSegment(back,3,0)==FollowResult::Invalid);
  assert(f.followSegment(back,3,2)==FollowResult::Invalid);
  PathPoint bad[]={{0,0,30},{0,8,std::numeric_limits<double>::quiet_NaN()}};
  assert(f.followSegment(bad,2,0.3)==FollowResult::Invalid);
  o.y=20;
  assert(f.followSegment(back,3,0.3)==FollowResult::Invalid); // disconnected

  // Curve command signs: reverse uses swapped, negated virtual wheels.
  o.y=0; l.first=r.first=0;
  PathPoint curve[]={{0,0,30},{2,4,30},{4,8,0}};
  assert(f.followSegment(curve,3,0.3,PathDirection::Reverse)==FollowResult::Timeout);
  assert(l.first<0 && r.first<0 && l.first>r.first);

  // A nearby passed endpoint must fail rather than orbit indefinitely.
  o.y=2.5; o.heading=0;
  PathPoint passed[]={{0,0,30},{0,5,30},{0,0,0}};
  assert(f.followSegment(passed,3,0.3,PathDirection::Forward)==FollowResult::MissedEndpoint);
  assert(vex::left==0 && vex::right==0);

  // A frozen follower exits after 1 s, braking, not its long total timeout.
  o.x=o.y=0; o.heading=0; o.frozen=true;
  PathPoint long_path[]={{0,0,30},{0,20,30},{0,40,30},{0,50,0}};
  double began=vex::now;
  f.follow(long_path,4,0.3,-1,1000);
  assert(vex::now-began>=1000 && vex::now-began<=1010);
  assert(vex::left==0 && vex::right==0);

  // A progressing run may last much longer than the 1-s stall timeout.
  o.y=0; o.frozen=false; began=vex::now;
  f.follow(long_path,4,0.3,-1,1000);
  assert(vex::now-began>1000 && o.y>=38);

  // Legacy follow keeps the last-positive-speed stopping rule.
  o.y=0; o.heading=0; o.frozen=false;
  PathPoint legacy[]={{0,0,30},{0,4,30},{0,8,0},{0,28,0}};
  f.follow(legacy,4,0.3);
  assert(o.y<6);
}
"""


def main():
    compiler = shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        raise SystemExit("clang++ or g++ is required")
    with tempfile.TemporaryDirectory() as directory:
        temp = Path(directory)
        (temp / "MotionLib").mkdir()
        (temp / "vex.h").write_text(VEX)
        (temp / "MotionLib/Odom.hpp").write_text(ODOM)
        (temp / "test.cpp").write_text(TEST)
        subprocess.run([compiler, "-std=c++17", "-I" + str(temp),
                        "-I" + str(ROOT / "include"), str(temp / "test.cpp"),
                        "-o", str(temp / "test")], check=True)
        subprocess.run([str(temp / "test")], check=True)
    print("PASS: reverse/forward, zero endpoint, braking, Auto ambiguity, "
          "timeout, validation, reverse steering, missed endpoint, legacy stopping")


if __name__ == "__main__":
    main()
