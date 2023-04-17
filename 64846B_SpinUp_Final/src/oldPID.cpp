#include "oldPID.h"
#include "vex.h"
#include "PID.h"
#include "GlobalVariables.h"


motor_group LeftDriveSmartO = motor_group(FrontLeft, BackLeft);
motor_group FRBLStrafeO = motor_group(FrontRight,BackLeft);
motor_group FLBRStrafeO = motor_group(FrontLeft,BackRight);
motor_group RightDriveSmartO = motor_group(FrontRight, BackRight);

smartdrive DrivetrainO = smartdrive(LeftDriveSmartO, RightDriveSmartO, TurnGyroSmart, 319.19, 320, 40, mm, 1);


double degDistanceO = 0;
//double pi = 3.14159;
double wheelDiameterO = 2.75;



// Used to count the number of turns for output data only
int turnCountO = 0;
// Relative degree tracking
int angleTrackerO = 0;
// Weighted factor of porportion error

//kd=.1;
// Max speed in Volts for motors
double maxSpeedO = 8;
// The angle difference from error when integral adjustments turns on

// Tolerance for approximating the target angle
double turnToleranceO = .5;
// Total number of iterations to exit while loop
int maxIterO = 700;
 //Keeps track of how many times angleTracker goes over 360
  
  double sensorT = 0;
void driveTo (double targetDistance, char driveDirection) {

if (driveDirection== 'S') {
sensorT = Back;
} else if (driveDirection == 'N') {
  sensorT = Right;
}
  drivePID.threshold = 9;
drivePID.kI = 0.04;

drivePID.kD = 0.03;

drivePID.kP = 0.15;
  //reset motor encoders so they are all zero
  //this way the ticks match up without worrying about turning

//declaration of local variables

drivePID.prevError = 0;
drivePID.derivative = 0;
drivePID.error = 0;
drivePID.integral = 0;

//converting target distance into ticks
degDistanceO = fabs(targetDistance / (wheelDiameterO * M_PI) *360);
double wheelConstant = wheelDiameterO * M_PI;

//while loop
//checks desired distance against sensor of current distance driven
// 10 allows us to have a threshold for ticks so if its close enough it stops but check math in case 10 is too high
//Check if threshold and this threshold need to be same
     while (fabs(degDistanceO) > (fabs(sensorT * 360 / wheelConstant)) || (fabs(degDistanceO) - (fabs(sensorT * 360 / wheelConstant) > 5))) {
//error is tick distance - sensor
    drivePID.error = degDistanceO - (fabs(sensorT) * 360 / wheelConstant);
//assign derivative 
    drivePID.derivative = drivePID.error - drivePID.prevError;
    //assign previous error as the error before
    drivePID.prevError = drivePID.error;

//if error is less than threshold and error is not 0 add integral + error to integral
if (fabs(drivePID.error) < drivePID.threshold && drivePID.error != 0) {
  drivePID.integral += drivePID.error;
  //else nothing
} else {
  drivePID.integral = 0;
}
//end of if
//ad switch for character to equal which sensor works

//declare and assign powerdrive (I.E. velocity control PID)
   drivePID.powerDrive = (drivePID.error * drivePID.kP) + (drivePID.derivative * drivePID.kD) + (drivePID.integral * drivePID.kI);
switch (driveDirection) {
  case 'l':
  if (targetDistance > 0) {
  FRBLStrafeO.spin(forward,drivePID.powerDrive,voltageUnits::volt);
   } else if(targetDistance <0){
  FRBLStrafeO.spin(reverse,drivePID.powerDrive,voltageUnits::volt);
   }
  break;
  case 'r':
  if (targetDistance > 0) {
    FLBRStrafeO.spin(forward,drivePID.powerDrive,voltageUnits::volt);
  } else if (targetDistance < 0) {
    FLBRStrafeO.spin(reverse,drivePID.powerDrive,voltageUnits::volt);
  }
  break;
  case 'N':
  if(targetDistance > 0) {
    LeftDriveSmartO.spin(forward,drivePID.powerDrive,voltageUnits::volt);
    RightDriveSmartO.spin(forward,drivePID.powerDrive,voltageUnits::volt);
  } else if (targetDistance < 0) {
    LeftDriveSmartO.spin(reverse,drivePID.powerDrive,voltageUnits::volt);
    RightDriveSmartO.spin(reverse,drivePID.powerDrive,voltageUnits::volt);
  }
  break;
  case 'S':
  if(targetDistance > 0) {
    FLBRStrafeO.spin(reverse,drivePID.powerDrive,voltageUnits::volt);
    FRBLStrafeO.spin(forward,drivePID.powerDrive,voltageUnits::volt);
  } else if (targetDistance < 0) {
    FLBRStrafeO.spin(forward,drivePID.powerDrive,voltageUnits::volt);
    FRBLStrafeO.spin(reverse,drivePID.powerDrive,voltageUnits::volt);
  }
  break;
  default:
  LeftDriveSmartO.stop();
  RightDriveSmartO.stop();

  
    this_thread::sleep_for(15);
    }//end of while loop
    
    //tell motors to stop if target is achieved
LeftDriveSmartO.stop();
RightDriveSmartO.stop();

//print data and assign last values
    drivePID.error = degDistanceO - (fabs(sensorT) * 360 / wheelConstant);
    drivePID.derivative = drivePID.error - drivePID.prevError;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.setCursor(1, 13);
  Controller1.Screen.newLine();
  Controller1.Screen.print("error: %.5f", drivePID.error);
  Controller1.Screen.newLine();
  Controller1.Screen.print("derivative %.5f", drivePID.error);
  Controller1.Screen.newLine();
}
}





// Used to count the number of turns for output data on
// Weighted factor of porportion error
double kPT = 0.15;
//kp=.18
// Weighted factor of integral error
double kIT = 0.009;
//ki=.008;
// Weighted factor of the derivated error
double kDT = .001;

//kd=.1;
// Max speed in Volts for motors
int turnThresholdO = 16;

// Turning Function
void turnToO(double angleTurn) {
  //  Distance to target in degrees
  double error = 0;
  //  Error degree from the last iteration
  double prevError = 0;
  // Derivative of the error. The slope between iterations
  double derivative = 0;
  // Accumulated error after threashold. Summing error
  double integral = 0;
  // Iterations of the loop. Counter used to exit loop if not converging
  double iter = 0;
 
 

  // Used for Relative Coordinates. For absolute coordinates, comment out
  // following lines for angleTracker
  //angleTracker += angleTurn;
  /*
  if (angleTracker > 360){
    modTracker += 1;
  }
  else if (angleTracker < -360){
    modTracker -= 1;
  }
  angleTurn = angleTracker - (modTracker*360);
*/
/*
  if (angleTurn > 360){
    modTracker += 1;
  }
  else if (angleTurn < -360){
    modTracker -= 1;
  }
  angleTurn = angleTracker - (modTracker*360);
  */

  // Automated error correction loop
  while (fabs(TurnGyroSmart.rotation(degrees) - angleTurn) > turnToleranceO && iter < maxIterO) 
  {
    iter += 1;
    error = angleTurn - TurnGyroSmart.rotation(degrees);
    /*if (error<-180) {
      error +=360;
    } else if (error>180) {
      error -=360;
    }*/
    derivative = error - prevError;
    prevError = error;

    // Checking if error passes threshold to build the integral
    if (fabs(error) < turnThresholdO && error != 0) 
    {
      integral += error;
    } else {
      integral = 0;
    }

    // Voltage to use. PID calculation
    double powerDrive = error * kPT + derivative * kDT + integral * kIT;

    // Capping voltage to max speed
    if (powerDrive > maxSpeedO) 
    {
      powerDrive = maxSpeedO;
    } 
    else if (powerDrive < -maxSpeedO) 
    {
      powerDrive = -maxSpeedO;
    }

    // Send to motors
    LeftDriveSmartO.spin(forward, powerDrive, voltageUnits::volt);
    RightDriveSmartO.spin(forward, -powerDrive, voltageUnits::volt);

    this_thread::sleep_for(15);
  }

  // Angle achieved, brake robot
  LeftDriveSmartO.stop(brake);
  RightDriveSmartO.stop(brake);

  // Tuning data, output to screen
  turnCountO += 1;
  error = angleTurn - TurnGyroSmart.rotation(degrees);
  derivative = error - prevError;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Turn #: %d", turnCountO);
  Controller1.Screen.setCursor(1, 13);
  Controller1.Screen.print("iter: %.0f", iter);
  Controller1.Screen.newLine();
  Controller1.Screen.print("error: %.5f", error);
  Controller1.Screen.newLine();
  Controller1.Screen.print("derivative: %.5f", derivative);
  Controller1.Screen.newLine();
}
