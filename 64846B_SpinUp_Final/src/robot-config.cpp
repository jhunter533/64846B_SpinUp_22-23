#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FrontLeft = motor(PORT12, ratio18_1, false);
motor BackLeft = motor(PORT15, ratio18_1, true);
motor FrontRight = motor(PORT20, ratio18_1, true);
motor BackRight = motor(PORT16, ratio18_1, false);
motor Intake = motor(PORT9,ratio6_1,false);
motor Indexer = motor(PORT8,ratio18_1,true);
motor Flywheel = motor(PORT2,ratio6_1,false);
controller Controller2 = controller(partner);
inertial TurnGyroSmart = inertial(PORT19);
triport ThreeWirePort=triport(PORT22);
encoder Right = encoder(ThreeWirePort.E);
encoder Back = encoder(ThreeWirePort.A);
digital_out Pneumatic = digital_out(Brain.ThreeWirePort.B);
digital_out Pneumatic2 = digital_out(Brain.ThreeWirePort.C);
motor_group LeftDriveSmart = motor_group(BackLeft,FrontLeft);
motor_group RightDriveSmart = motor_group(BackRight,FrontRight);
motor Roller = motor(PORT10,ratio36_1,false);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1.4);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  TurnGyroSmart.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (TurnGyroSmart.isCalibrating()) {
    wait(250, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}