"""Compile real Drive/Odom/Pid sources against a minimal host VEX model.
Run with python3 tests/test_straight_move.py; requires clang++ or g++.
This tests controller logic, not physical chassis performance.
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
enum units { msec, sec, forward, pct, brake, deg, degrees=deg, volt };
inline double now=0, left=0, right=0, pod=0, heading_value=0, gain=0.6;
inline bool frozen=false;
inline void wait(double ms, units) {
  if (!frozen) {
    pod+=(left+right)*gain/2*ms/1000*360/(2*M_PI);
    heading_value+=(left-right)*gain/13.15*ms/1000*180/M_PI;
  }
  now+=ms;
}
struct rotation {
  bool y;
  double position(units) {return y?pod:0;}
  void resetPosition() {if(y)pod=0;}
};
struct inertial { double heading(units) {return heading_value;} };
struct motor_group {
  bool is_left; double first=0, peak=0;
  void spin(units,double value,units command_units) {
    if(first==0 && value!=0)first=value;
    peak=fmax(peak,fabs(value));
    (is_left?left:right)=command_units==volt ? value*100.0/12.0 : value;
  }
  void stop(units) {(is_left?left:right)=0;}
};
struct brain {struct screen {
  template<class... T> void printAt(int,int,const char*,T...) {}
} Screen;};
struct timer {
  double start=now;
  void reset(){start=now;}
  double time(units u){return(now-start)/(u==sec?1000:1);}
};
}
"""
TEST = r"""
#include "MotionLib/Drive.hpp"
#include <cassert>
#include <limits>
bool never(){return false;}
double cancel_at;
bool until(){return vex::now<cancel_at;}
void stopped(){assert(vex::left==0 && vex::right==0);}
int main(){
  using motion::MoveResult;
  vex::motor_group left{true},right{false};
  vex::rotation y{true},x{false};vex::inertial imu;vex::brain brain;
  motion::Drive drive(left,right,y,x,imu,brain,2.0,13.15);
  drive.init();
  assert(drive.moveStraight(28)==MoveResult::Reached);
  assert(drive.get_y()>=26.5 && drive.get_y()<28);stopped();
  assert(left.peak<=60 && right.peak<=60);
  drive.init();left.first=right.first=0;
  assert(drive.moveStraight(-17)==MoveResult::Reached);
  assert(drive.get_y()<=-15.5 && left.first<0 && right.first<0);stopped();
  drive.init();double start=vex::now;
  assert(drive.moveStraight(0)==MoveResult::Reached);
  assert(vex::now==start);stopped();
  assert(drive.moveStraight(10,0)==MoveResult::Invalid);stopped();
  assert(drive.moveStraight(10,101)==MoveResult::Invalid);stopped();
  assert(drive.moveStraight(std::numeric_limits<double>::quiet_NaN())==MoveResult::Invalid);stopped();
  assert(drive.moveStraight(10,60,never)==MoveResult::Cancelled);stopped();
  cancel_at=vex::now+50;
  assert(drive.moveStraight(28,60,until)==MoveResult::Cancelled);stopped();
  drive.init();vex::frozen=true;start=vex::now;
  assert(drive.moveStraight(28)==MoveResult::Stalled);
  assert(vex::now-start==500);stopped();
  drive.init();vex::frozen=false;vex::gain=0.01;start=vex::now;
  assert(drive.moveStraight(28)==MoveResult::Timeout);
  assert(vex::now-start>=3240);stopped();
  drive.init();vex::gain=0.6;left.peak=right.peak=0;
  auto result=drive.moveStraight(3,6);
  assert(result==MoveResult::Reached || result==MoveResult::Stalled);
  assert(left.peak<=6 && right.peak<=6);stopped();
  // Friendly configuration: rejected settings leave prior values intact.
  drive.init();assert(drive.setMoveTolerance(3));assert(!drive.setMoveTolerance(0));
  start=vex::now;assert(drive.moveStraight(2)==MoveResult::Reached);assert(vex::now==start);
  assert(drive.setMoveTolerance(1.5));
  assert(drive.setStallTimeout(200));assert(!drive.setStallTimeout(-1));
  vex::frozen=true;start=vex::now;
  assert(drive.moveStraight(10)==MoveResult::Stalled);assert(vex::now-start==200);
  assert(drive.setStallTimeout(0));start=vex::now;
  assert(drive.moveStraight(10)==MoveResult::Timeout);assert(vex::now-start>=1800);
  assert(drive.setStallTimeout(500));vex::frozen=false;
  assert(!drive.shouldContinue(MoveResult::Timeout));
  drive.setFailurePolicy(motion::FailurePolicy::Continue);
  assert(drive.shouldContinue(MoveResult::Stalled));
  assert(drive.shouldContinue(motion::FollowResult::Timeout));
  assert(!drive.shouldContinue(MoveResult::Invalid));
  assert(!drive.shouldContinue(MoveResult::Cancelled));
  assert(!drive.shouldContinue(motion::FollowResult::MissedEndpoint));

  drive.init();left.first=right.first=0;start=vex::now;
  assert(drive.forwardFor(75,25)==MoveResult::Reached);
  assert(vex::now-start==25 && left.first==75 && right.first==75);stopped();
  left.first=right.first=0;
  assert(drive.backwardFor(40,20)==MoveResult::Reached);
  assert(left.first==-40 && right.first==-40);stopped();
  assert(drive.forwardFor(10,-1)==MoveResult::Invalid);stopped();
  assert(drive.forwardFor(101,20)==MoveResult::Invalid);stopped();
  assert(drive.moveBackward(-5)==MoveResult::Invalid);stopped();
  assert(drive.moveForward(-5)==MoveResult::Invalid);stopped();

  // Relative turns preserve pose frame; legacy turn methods still link.
  drive.init(0,0,90);left.first=right.first=0;
  auto turn_result=drive.turnBy(-18,25);
  assert(turn_result==MoveResult::Reached || turn_result==MoveResult::Timeout);
  assert(drive.get_theta()<90 && left.first<0 && right.first>0);stopped();
  assert(drive.turnBy(361)==MoveResult::Invalid);stopped();
  assert(drive.turn(90,0)==MoveResult::Invalid);stopped();
  drive.turnTo(drive.get_theta(),25);stopped();
  drive.turnLeft(drive.get_theta(),25);stopped();
  drive.turnRight(drive.get_theta(),25);stopped();

  drive.setContinueCondition(never);
  assert(drive.moveForward(10)==MoveResult::Cancelled);stopped();
  assert(drive.backwardFor(40,20)==MoveResult::Cancelled);stopped();
  assert(drive.turnBy(18)==MoveResult::Cancelled);stopped();
  PathPoint path[]={{0,0,30},{0,10,30},{0,12,0}};
  assert(drive.followPath(path,3)==motion::FollowResult::Cancelled);stopped();
  drive.setContinueCondition(nullptr);
  drive.init();vex::frozen=true;start=vex::now;
  assert(drive.followPath(path,3,0.8,-1,100)==motion::FollowResult::Stalled);
  assert(vex::now-start==100);stopped();
  assert(drive.followPath(nullptr,3)==motion::FollowResult::Invalid);stopped();
  drive.setContinueCondition(until);cancel_at=vex::now+20;
  assert(drive.forwardFor(40,100)==MoveResult::Cancelled);stopped();
  drive.setContinueCondition(nullptr);vex::frozen=false;

  // Manual driving ignores the autonomous callback, preserving arcade mix.
  drive.init();drive.setContinueCondition(never);
  assert(drive.arcade(100,100));
  assert(drive.get_left_command_volts()==12 && drive.get_right_command_volts()==0);
  assert(drive.arcade(60,40));
  assert(drive.get_left_command_volts()==12);
  assert(fabs(drive.get_right_command_volts()-20.0/8.3)<1e-9);
  assert(drive.arcade(-100,-100));
  assert(drive.get_left_command_volts()==-12 && drive.get_right_command_volts()==0);
  assert(drive.arcade(1,0)); // no new deadband
  assert(fabs(drive.get_left_command_volts()-1.0/8.3)<1e-9);
  assert(drive.arcade(50,0));vex::wait(20,vex::msec);
  assert(drive.arcade(0,0));assert(drive.get_y()>0); // updates odometry
  assert(!drive.arcade(std::numeric_limits<double>::quiet_NaN(),0));stopped();
  assert(drive.get_left_command_volts()==0 && drive.get_right_command_volts()==0);
  drive.setContinueCondition(nullptr);

  drive.init();vex::pod=std::numeric_limits<double>::quiet_NaN();
  assert(drive.moveStraight(10)==MoveResult::Invalid);stopped();
}
"""


def main():
    compiler = shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        raise SystemExit("clang++ or g++ is required")
    with tempfile.TemporaryDirectory() as directory:
        temp = Path(directory)
        (temp / "vex.h").write_text(VEX)
        (temp / "test.cpp").write_text(TEST)
        subprocess.run([compiler, "-std=c++17", "-I" + str(temp),
                        "-I" + str(ROOT / "include"), str(temp / "test.cpp"),
                        *[str(ROOT / "src" / name) for name in
                          ("Drive.cpp", "Odom.cpp", "Pid.cpp")],
                        "-o", str(temp / "test")], check=True)
        subprocess.run([str(temp / "test")], check=True)
    print("PASS: forward/reverse, tolerance, speed caps, invalid input/feedback, "
          "cancellation, stall/total timeout, settings, policy, timed/relative moves, "
          "path results and legacy compatibility; all exits brake")


if __name__ == "__main__":
    main()
