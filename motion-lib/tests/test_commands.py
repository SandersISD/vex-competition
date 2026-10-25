"""Host tests for the reusable runner using real MotionLib controllers.
Run: python3 tests/test_commands.py. Not a physical accuracy test.
"""
from pathlib import Path
import runpy
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
VEX = runpy.run_path(str(ROOT / "tests/test_straight_move.py"))["VEX"]
TEST = r"""
#include "MotionLib/Commands.hpp"
#include <cassert>
#include <limits>
vex::motor_group left{true},right{false};vex::rotation y{true},x{false};
vex::inertial imu;vex::brain brain;
motion::Drive drive(left,right,y,x,imu,brain,2.0,13.15);
bool allowed=true;double cancelAt=1e99;
int failed=0,clamps=0,actions=0,messages=0;double height=0;
bool permitted(){return allowed && vex::now<cancelAt;}
void onFailure(){++failed;}
void message(const char*){++messages;}
void clamp(){++clamps;}
void lift(double target,double speed){assert(speed==80);height=target;++actions;}
bool good(){++actions;return true;}
bool bad(){++actions;return false;}
void disable(){allowed=false;}
double mirror(double h){return fmod(180-h+360,360);}
void reset(){
 allowed=true;cancelAt=1e99;failed=clamps=actions=messages=0;height=0;
 vex::frozen=false;vex::gain=0.6;left.first=right.first=0;
 drive.setContinueCondition(permitted);
 drive.setFailurePolicy(motion::FailurePolicy::Continue);
 drive.setMoveTolerance(1.5);drive.setStallTimeout(500);drive.init();
}
int main(){
 using motion::Commands;
 reset();
 {Commands a(drive,onFailure,message);a.forward(-1);a.action(clamp);
  a.action(lift,500.0,80.0);a.backward(10);a.finish();assert(a.failed()&&!a.finished());}
 assert(clamps==0 && actions==0 && failed==1);assert(vex::left==0 && vex::right==0);
 reset();
 {Commands a(drive,onFailure);a.action(bad);a.action(clamp);a.wait(100);}
 assert(actions==1 && clamps==0 && failed==1);
 reset();
 {Commands a(drive,onFailure);a.action(lift,1000.0,80.0);a.action(good);
  a.action(clamp);a.finish();a.action(clamp);assert(a.finished()&&!a.failed());}
 assert(height==1000 && actions==2 && clamps==1 && failed==0);
 reset();allowed=false;
 {Commands a(drive,onFailure);a.action(clamp);a.forwardFor(75,500);a.initPose(1,2,3);}
 assert(clamps==0 && failed==1 && left.first==0);
 reset();
 {Commands a(drive,onFailure);a.action(disable);a.action(clamp);}
 assert(clamps==0 && failed==1);
 reset();vex::frozen=true;
 {Commands a(drive,onFailure);a.forward(10);a.action(clamp);assert(!a.failed());}
 assert(clamps==1 && failed==0); // intended continuation after stall
 reset();vex::frozen=true;drive.setFailurePolicy(motion::FailurePolicy::Stop);
 {Commands a(drive,onFailure);a.forward(10);a.action(clamp);}
 assert(clamps==0 && failed==1);
 reset();vex::frozen=true;
 {Commands a(drive,onFailure);a.turnTo(90);a.action(clamp);}
 assert(clamps==0 && failed==1); // heading guard overrides timeout Continue
 reset();vex::frozen=true;
 {Commands a(drive,onFailure);a.turnBy(360);a.action(clamp);}
 assert(clamps==0 && failed==1); // relative full turn cannot pass via modulo 360
 reset();vex::frozen=true;
 {Commands a(drive,onFailure);a.setHeadingTransform(mirror);a.turnTo(180);a.action(clamp);}
 assert(clamps==1 && failed==0);
 reset();cancelAt=vex::now+20;
 {Commands a(drive,onFailure);a.wait(100);a.action(clamp);}
 assert(clamps==0 && failed==1);
 reset();
 {Commands a(drive,onFailure);a.follow(nullptr,2);a.action(clamp);}
 assert(clamps==0 && failed==1);
 reset();
 {Commands a(drive,onFailure);void(*empty)()=nullptr;a.action(empty);a.action(clamp);}
 assert(clamps==0 && failed==1);
 reset();
 {Commands a(drive,onFailure);assert(!a.setTurnTolerance(0));
  assert(!a.setTurnTolerance(std::numeric_limits<double>::quiet_NaN()));
  a.abort();a.abort();a.action(clamp);}
 assert(clamps==0 && failed==1);
 reset();
 {Commands a(drive,onFailure);a.forwardFor(75,25);a.backwardFor(40,25);
  a.wait(25);a.action(clamp);a.finish();assert(a.finished());}
 assert(clamps==1 && failed==0 && vex::left==0 && vex::right==0);
 reset();
 {Commands a(drive,onFailure);a.action([&](){++actions;});a.finish();}
 assert(actions==1 && failed==0);
 reset();
 {Commands a(drive,onFailure);
  assert(a.setMoveTolerance(3.0));assert(!a.setMoveTolerance(0));
  assert(a.setStallTimeout(200));assert(!a.setStallTimeout(-1));
  a.setFailurePolicy(motion::FailurePolicy::Continue);
  vex::frozen=true;double began=vex::now;a.forward(10);a.action(clamp);
  assert(vex::now-began==200 && clamps==1 && !a.failed());}
 reset();
 {Commands a(drive,onFailure);a.setContinueCondition(permitted);
  allowed=false;a.action(clamp);assert(a.failed());}
 assert(clamps==0 && failed==1);
}
"""


def main():
    compiler = shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        raise SystemExit("clang++ or g++ required")
    with tempfile.TemporaryDirectory() as directory:
        temp = Path(directory)
        (temp / "vex.h").write_text(VEX)
        (temp / "test.cpp").write_text(TEST)
        subprocess.run([compiler, "-std=c++17", "-I" + str(temp),
                        "-I" + str(ROOT / "include"), str(temp / "test.cpp"),
                        *[str(ROOT / "src" / f) for f in
                          ("Drive.cpp", "Odom.cpp", "Pid.cpp")],
                        "-o", str(temp / "test")], check=True)
        subprocess.run([str(temp / "test")], check=True)
    print("PASS: sequence latch, mechanism callbacks, continuation, heading guard, "
          "mirror, relative turn, cancellation, null action, abort and finish")


if __name__ == "__main__":
    main()
