"""Test the actual driver update functions with host-side hardware fakes.
Run: python3 tests/test_driver_updates.py (clang++ or g++ required).
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PREFIX = r"""
#include <cassert>
#include <cmath>
namespace vex {
enum units{pct,hold,msec,brake,deg,forward,reverse};
inline double now=0;
void wait(double t,units){now+=t;}
struct timer{double start=now;void reset(){start=now;}double time(units){return now-start;}};
}
struct Button{bool value=false;bool pressing(){return value;}};
struct Axis{double value=0;double position(){return value;}};
struct ControllerType{Button ButtonB,ButtonX,ButtonDown,ButtonL1,ButtonUp,ButtonR1,ButtonR2;Axis Axis3,Axis1;}Controller;
struct Motor{
 bool done=false;double pos=0,target=0;int moves=0,stops=0,spins=0;vex::units direction=vex::forward;
 void setVelocity(double,vex::units){}void setStopping(vex::units){}
 void spinToPosition(double p,vex::units,bool){target=p;done=false;++moves;}
 void spinFor(vex::units d,double p,vex::units,bool){target=p;direction=d;done=false;++moves;}
 void spin(vex::units d){direction=d;++spins;}
 void stop(){++stops;}void stop(vex::units){++stops;}
 bool isDone(){return done;}double position(vex::units){return pos;}
}Cascade,SideToggle,CascadeRotation,LeftToggleRotation,RightToggleRotation;
struct ClawType{bool value=false;int commands=0;void set(bool v){value=v;++commands;}}Claw;
struct BrainType{struct ScreenType{template<class... T>void printAt(int,int,const char*,T...) {}}Screen;}Brain;
struct FakeDrive{
 void arcade(double,double){}double get_left_command_volts(){return 0;}
 double get_right_command_volts(){return 0;}double get_x(){return 0;}
 double get_y(){return 0;}double get_theta(){return 0;}
}drive;
struct CompetitionType{bool isEnabled(){return false;}bool isDriverControl(){return false;}}Competition;
constexpr int MECHANISM_TIMEOUT_MS=2000;
bool toggleFault=false;double toggleCounter=0;int stopCount=0;
void stopAllMotors(){++stopCount;}
"""
TEST = r"""
void reset(){
 Controller=ControllerType{};Cascade=Motor{};SideToggle=Motor{};
 CascadeRotation=Motor{};LeftToggleRotation=Motor{};RightToggleRotation=Motor{};
 Claw=ClawType{};toggleFault=false;toggleCounter=0;vex::now=0;beginDriverControl();
}
void tick(int milliseconds=100){vex::wait(milliseconds,vex::msec);updateClawButtons();updateCascade();}
int main(){
 reset();Controller.ButtonDown.value=true;updateClawButtons();
 assert(Claw.value && Claw.commands==1);updateClawButtons();assert(Claw.commands==1);
 Controller.ButtonB.value=true;updateClawButtons();assert(!Claw.value);
 tick();assert(driverCascadeState==CASCADE_B_DELAY && Cascade.moves==0);
 tick();assert(Cascade.moves==0);tick();assert(Cascade.target==100 && Cascade.moves==1);
 Controller.ButtonR1.value=true;Controller.ButtonB.value=false;
 tick();assert(Cascade.spins==0); // presets retain priority over manual raise
 Cascade.done=true;tick();assert(driverCascadeState==CASCADE_MANUAL);
 tick();assert(Cascade.spins==1 && Cascade.direction==vex::forward);
 reset();Controller.ButtonB.value=true;Controller.ButtonUp.value=true;tick();
 assert(Cascade.target==0 && Claw.value); // Up wins cascade branch after B claw event
 reset();Controller.ButtonR2.value=true;CascadeRotation.pos=-1;tick();assert(Cascade.spins==0);
 CascadeRotation.pos=0;tick();assert(Cascade.spins==1 && Cascade.direction==vex::reverse);
 reset();Controller.ButtonX.value=true;updateClawButtons();assert(!Claw.value);
 reset();Controller.ButtonL1.value=true;updateSideToggle();
 assert(SideToggle.moves==1 && SideToggle.target==180 && toggleCounter==180);
 updateSideToggle();assert(SideToggle.moves==1); // held button cannot retrigger
 LeftToggleRotation.pos=-170;RightToggleRotation.pos=170;SideToggle.done=true;
 updateSideToggle();assert(SideToggle.moves==2 && SideToggle.target==10);
 assert(driverToggleState==TOGGLE_CORRECTION && SideToggle.direction==vex::reverse);
 SideToggle.done=true;updateSideToggle();assert(driverToggleState==TOGGLE_IDLE);
 reset();Controller.ButtonL1.value=true;updateSideToggle();vex::wait(2000,vex::msec);
 updateSideToggle();assert(toggleFault && driverToggleState==TOGGLE_IDLE);
 Controller.ButtonL1.value=false;updateSideToggle();Controller.ButtonL1.value=true;
 updateSideToggle();assert(SideToggle.moves==1);
 beginDriverControl();assert(toggleFault && toggleCounter==180); // persistent references
 reset();Controller.ButtonB.value=true;beginDriverControl();updateClawButtons();
 assert(Claw.commands==0); // mode-entry held button is not a fresh press
 stopCount=0;driverControl();assert(stopCount==1); // exits stopped when disabled
}
"""


def main():
    source = (ROOT / "src/main.cpp").read_text()
    start = source.index("// Driver update state.")
    end = source.index("// ------------------------------------------------------------------ main", start)
    compiler = shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        raise SystemExit("clang++ or g++ required")
    with tempfile.TemporaryDirectory() as directory:
        temp = Path(directory)
        (temp / "test.cpp").write_text(PREFIX + source[start:end] + TEST)
        subprocess.run([compiler, "-std=c++17", str(temp / "test.cpp"),
                        "-o", str(temp / "test")], check=True)
        subprocess.run([str(temp / "test")], check=True)
    print("PASS: claw edges, B delay, cascade priority/lower limit, "
          "toggle correction/fault latch, mode reset and stopped exit")


if __name__ == "__main__":
    main()
