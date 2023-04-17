/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jessica Hunter                                            */
/*    Created:      November 21 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// FrontLeft            motor         12
// BackLeft             motor         15
// FrontRight           motor         20
// BackRight            motor         16
// Intake               motor         9
// Roller               motor         10
// Indexer              motor         8
// Flywheel             motor         2
// TurnGyroSmart        inertial      19
// Pneumatic            three wire    B
// Pneumatic2           three wire    C
// LeftDriveSmart       motor group   12, 15
// RightDriveSmart      motor group   20, 16
// Drivetrain           smartdrive
// Controller2          controller
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"
#include "PID.h"
//Odometry no longer used
#include "Odometry.h"
#include "GlobalVariables.h"
#include "Conversion.h"
#include <iostream>
//Graphics for odometry no longer used
#include "Graphics.h"
//TBH algorithm no longer used due to time constraint
#include "TBH.h"

using namespace vex;
task TrackPositionTask;
task printTask;
task flywheelTask;
task VisionTask;

int printRPM(){
  while(1){
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("MRPM: %.5f", Flywheel.velocity(rpm));
        Controller2.Screen.setCursor(1, 1);
        Controller2.Screen.print("MRPM: %.5f", Flywheel.velocity(rpm));
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("WRPM: %.5f", Flywheel.velocity(rpm)*6);
        Controller2.Screen.setCursor(2, 1);
        Controller2.Screen.print("WRPM: %.5f", Flywheel.velocity(rpm)*6);
        wait(.2,sec);
        task::sleep(20);
  }return 1;
}




//motor_group LeftDriveSmart = motor_group(FrontLeft, BackLeft);

//motor_group RightDriveSmart = motor_group(FrontRight, BackRight);

//smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, TurnGyroSmart, 319.19, 320, 40, mm, 1.4);


//////////////PID Turning////////////////////////////////////////////////////////
// PID = Porportion, Inegral, Deriviative (Tuning Parameters)
// Porportion: Distance to target angle
// Inegral: Acumulated error within the threshold
// Derivative: Rate of change of the error (Note: not needed for our purposes)

// Guidelines for Tuning:
//    - First: Baseline your parameters to learn the right values for Your robot
//        - set kP = 0.1 (this seems to be. a good place to start)
//        - set kI and kD to 0 (noting kD will not change)
//        - set maxSpeed (noting the higher maxSpeed is the greater the initial
//        rotation,
//                          but may be more erratic)
//        - set turnThreshold = 2 (something small enough to learn the
//        appropriate threshold)
//        - set turnTolerance = 1 (something reasonable that says we are close
//        enough)
//              (note: turnThreshold > turnTolerance, and they represent
//              boundaries on the error)
//        - set maxIter to something reasonably high (~ something in the 100s)
//    - Next: Learn the correct turnThreshold for Your Robot!!
//        - Note: at the end of the while loop, make sure the following is all
//        uncommented
//            - final error and derivative calculations
//            - print statements for iter, error, and derivative
//        - !! Download and Run !!
//            - We expect iter == maxIter (this means that the code terminated
//            without
//                meeting the expected turnTolerance.)
//        - Round up the error printed to the screen and set turnThreshold to
//        that value.
//        - Now start to slowly turn on kI <= kP
//        - !! Download and Run !!
//    - Finally: Tune kI
//        - If iter == maxIter
//            - increase kI
//        - If error < 0
//            - decrease kI
//********//Have fun tuning and learning how these parameters work
// together//********//

// Used to count the number of turns for output data only

int turnCountp = 0;
// Relative degree tracking
int angleTrackerp = 0;
// Weighted factor of porportion error
double kP = 0.163;
// Weighted factor of integral
double kI = 0.016;
// Weighted factor of the derivated error
double kD = 0.16;
// Max speed in Volts for motors
double maxSpeedp = 7;
// The angle difference from error when integral adjustments turns on
int turnThresholdp = 15;
// Tolerance for approximating the target angle
double turnTolerancep = .6;
// Total number of iterations to exit while loop
int maxIterp = 300;
// Turning Function
// Turns to absolute heading inputed
void turnPTo(double angleTurn) {
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
  /*
  //When previously enabled the robot would turn 90 to the right instead of to orientation 90
  //This meant that it would turn correctly until whatever angles that are adding equaled 0
  //Once that happened the robot would turn in a circle the opposite direction then turn to the angle going the other direction
  //While relative is easier for humans for a robot it makes it unreliable since every angle is built on the error of the previous

  angleTracker += angleTurn;
  angleTurn = angleTracker % 360;
*/
  // Automated error correction loop
  //Loop runs every 15 miliseconds
  //This function is in degrees internally and externally
  //while the error is less than thresholds and the amount of times in the loop is less than threshold
  // This is to account for when you haven't reached your target but the robot is stuck it may exit the loop
  while (fabs(TurnGyroSmart.rotation(degrees) - angleTurn) > turnTolerancep && iter < maxIterp) {
    iter += 1;
    //The error is the target angle - current angle 
    error = angleTurn - TurnGyroSmart.rotation(degrees);
    //The derivative is the error-previous error
    //These are defined as 0 before the loop runs
    derivative = error - prevError;
    //Then change the previous error value to the error before the next loop
    prevError = error;

    // Checking if error passes threshold to build the integral
    //The integral allows you to correct for overshooting so you don't want this to always run
    if (fabs(error) < turnThresholdp && error != 0) {
      integral += error;
    } else {
      integral = 0;
    }

    // Voltage to use. PID calculation
    double powerDrive = error * kP + derivative * kD + integral * kI;

    // Capping voltage to max speed
    if (powerDrive > maxSpeedp) {
      powerDrive = maxSpeedp;
    } else if (powerDrive < -maxSpeedp) {
      powerDrive = -maxSpeedp;
    }
    // Send voltage to motors
    LeftDriveSmart.spin(forward, powerDrive, voltageUnits::volt);
    RightDriveSmart.spin(forward, -powerDrive, voltageUnits::volt);

    this_thread::sleep_for(10);
    /*
    printf("%.5f",powerDrive);
    printf(" , ");
    printf("%.5f",iter);
    printf(" , ");
    printf("%.5f", angleTurn);
    printf(" , ");
    printf("%.5f", angleTurn-TurnGyroSmart.rotation(degrees));
    printf("\n");
    */
  }

  // Angle achieved, brake robot
  LeftDriveSmart.stop(brake);
  RightDriveSmart.stop(brake);

  // Tuning data, output to screen
  /*
  turnCountp += 1;
  error = angleTurn - TurnGyroSmart.rotation(degrees);
  derivative = error - prevError;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Turn #: %d", turnCountp);
  Controller1.Screen.setCursor(1, 13);
  Controller1.Screen.print("iter: %.0f", iter);
  Controller1.Screen.newLine();
  Controller1.Screen.print("error: %.5f", error);
  Controller1.Screen.newLine();
  Controller1.Screen.print("derivative: %.5f", derivative);
  Controller1.Screen.newLine();
  printf("%.5f \n", TurnGyroSmart.rotation(degrees));
  */
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                   */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// Creates a competition object that allows access to Competition methods.
vex::competition Competition;
/*                  GLOBAL DEFINITIONS
 *
 *      These global variables are used across this program to maintain state
 * (choices)
 *
 */
// storage for our auton selection
int autonomousSelection = -1;

// collect data for on screen button and include off and on color feedback for
// button pric - instead of radio approach with one button on or off at a time,
// each button has a state. ie shoot Preload may be low yellow and high yellow when on.
typedef struct _button {
  int xpos;
  int ypos;
  int width;
  int height;
  bool state;
  vex::color offColor;
  vex::color onColor;
  const char *label;
} button;

// Button array definitions for each software button. The purpose of each button
// data structure is defined above. The array size can be extended, so you can
// have as many buttons as you wish as long as it fits.
button buttons[] = {
    {30, 30, 60, 60, false, 0xE00000, 0x00E000, "Neutral"},
    {150, 30, 60, 60, false, 0xE00000, 0x00E000, "RightSide"},
    {270, 30, 60, 60, false, 0xE00000, 0x00E000, "skillsNew"},
    {390, 30, 60, 60, false, 0xE00000, 0x00E000, "No"},
    {30, 150, 60, 60, false, 0xE00000, 0x00E000, "skillsOld"},
    {150, 150, 60, 60, false, 0xE00000, 0x00E000, "skills6"},
    {270, 150, 60, 60, false, 0xE00000, 0x00E000, "RightFar"},
    {390, 150, 60, 60, false, 0xE00000, 0x00E000, "LeftFar"}};

// forward ref
void displayButtonControls(int index, bool pressed);

/*-----------------------------------------------------------------------------*/
/** @brief      Check if touch is inside button */
/*-----------------------------------------------------------------------------*/

int findButton(int16_t xpos, int16_t ypos) {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    button *pButton = &buttons[index];
    if (xpos < pButton->xpos || xpos > (pButton->xpos + pButton->width))
      continue;
      
    
    if (ypos < pButton->ypos || ypos > (pButton->ypos + pButton->height))
      continue;

    return (index);
  }
  return (-1);
}

/*-----------------------------------------------------------------------------*/
/** @brief      Init button states */
/*-----------------------------------------------------------------------------*/

void initButtons() {
  int nButtons = sizeof(buttons) / sizeof(button);

  for (int index = 0; index < nButtons; index++) {
    buttons[index].state = false;
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been touched */
/*-----------------------------------------------------------------------------*/

void userTouchCallbackPressed() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    displayButtonControls(index, true);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Screen has been (un)touched */
/*-----------------------------------------------------------------------------*/

void userTouchCallbackReleased() {
  int index;
  int xpos = Brain.Screen.xPosition();
  int ypos = Brain.Screen.yPosition();

  if ((index = findButton(xpos, ypos)) >= 0) {
    // clear all buttons to false, ie. unselected
    //      initButtons();

    // now set this one as true
    if (buttons[index].state == true) {
      buttons[index].state = false;
    } else {
      buttons[index].state = true;
    }

    // save as auton selection
    autonomousSelection = index;

    displayButtonControls(index, false);
  }
}

/*-----------------------------------------------------------------------------*/
/** @brief      Draw all buttons */
/*-----------------------------------------------------------------------------*/

void displayButtonControls(int index, bool pressed) {
  vex::color c;
  Brain.Screen.setPenColor(vex::color(0xe0e0e0));

  for (int i = 0; i < sizeof(buttons) / sizeof(button); i++) {

    if (buttons[i].state)
      c = buttons[i].onColor;
    else
      c = buttons[i].offColor;

    Brain.Screen.setFillColor(c);

    // button fill
    if (i == index && pressed == true) {
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
          buttons[i].width, buttons[i].height, c);
    } else
      Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
         buttons[i].width, buttons[i].height);

    // outline
    Brain.Screen.drawRectangle(buttons[i].xpos, buttons[i].ypos,
         buttons[i].width, buttons[i].height,
        vex::color::transparent);

    // draw label
    if (buttons[i].label != NULL)
      Brain.Screen.printAt(buttons[i].xpos + 8,
       buttons[i].ypos + buttons[i].height - 8,
        buttons[i].label);
  }
}

using namespace vex;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Back.resetRotation();
  Right.resetRotation();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// Autonomous function opns
void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  /* initialize capabilities from buttons */

  // Bool statements for when code is running to press before competition starts
  bool skills1 = buttons[0].state;
  bool skills2 = buttons[1].state;
  bool skills3 = buttons[2].state;
  bool skills4 = buttons[3].state;
  bool skills5 = buttons[4].state;
  bool skills6 = buttons[5].state;
  bool  skills7= buttons[6].state;
  bool skills8 = buttons[7].state;

  if (skills1) {

    //..........Starting Skills..........//
    //Neutral start for 15 sec
    //Shoot two preloads into low zone

    Flywheel.spin(forward,5,volt);
    wait(3,sec);
    Indexer.spinFor(360,deg,75,velocityUnits::pct);
    wait(.5,sec);
    Indexer.spinFor(360,deg,75,velocityUnits::pct);
    wait(2,sec);
    Flywheel.stop();
    //...............END OF CODE...............//
  }

  if (skills2) {

    //..........Starting..........//
    //Auton right roller
      //right side auton
          Flywheel.spin(forward,5,volt);
    //Spin up then launch two disks into starting lo zone
    wait(3,sec);
    Indexer.spinFor(360,deg,75,velocityUnits::pct);
    wait(.5,sec);
    Indexer.spinFor(360,deg,75,velocityUnits::pct);
    wait(.5,sec);
    Flywheel.stop();
   // turnPTo(180);
    //turnPTo(180);
    wait(.2,sec);
    Drivetrain.driveFor(52,inches,80,velocityUnits::pct);
    turnPTo(90);
    Roller.spin(forward,-100,velocityUnits::pct);
    Drivetrain.drive(forward,80,velocityUnits::pct);
    //After shooting disks score roller then end
    wait(.4,sec);
    Roller.stop();
    //..........Prepare for User Control..........//
  }

  if (skills3) {

    //..........Starting..........//
      //skills
      //Starting on the roller that you can start on the same tile
      //Spin roller as driving forward to turn roller to red
      Roller.spin(forward,100,percent);
      Drivetrain.drive(forward,100,velocityUnits::pct);
      wait(.37,sec);
      Roller.stop();
      //After roller is scored back away from wall and start spinning intake
      Drivetrain.driveFor(-1.3,inches,80,velocityUnits::pct);
      Intake.spin(forward,100,velocityUnits::pct);
      //Turn to face disk
      turnPTo(114);
      turnPTo(114);
      //Turn called twice in case the first does not turn enough
      Drivetrain.driveFor(9,inches,40,velocityUnits::pct);
      //Carrying two preloads drive forward while stopping to properly secure disk
      wait(.5,sec);
      Drivetrain.driveFor(7,inches,40,velocityUnits::pct);
      wait(.75,sec);
      Drivetrain.driveFor(12,inches,50,velocityUnits::pct);
      //Turn to face roller directly in front of
      turnPTo(90);
      turnPTo(90);
      //Repeat same roller spinning routine
      Roller.spin(forward,100,percent);
      Drivetrain.drive(forward,40,velocityUnits::pct);
      wait(.65,sec);
      Roller.stop();
      //While doing this the intake is still running just in case the disk isn't all the way secured
      Intake.stop();
      Drivetrain.driveFor(-1,inches,70,velocityUnits::pct);
      //Turn to face blue high goal 
      //Then drive forward to start shooting
      turnPTo(0.5);
      Drivetrain.driveFor(-36,inches,90,velocityUnits::pct);
      turnPTo(0.5);
      //Spin up flywheel to top speed and wait for speed to reach consistency
      Flywheel.spin(forward,11,volt);
      wait(3.7,sec);
      //At desired speed rotate indexer
      //This will push the disk into the flywheel shooting it
      Indexer.spinFor(forward,360,degrees,100,velocityUnits::pct);
      wait(1.6,sec);
      Indexer.spinFor(forward,360,degrees,100,velocityUnits::pct);
      wait(1.6,sec);
      Indexer.spinFor(forward,360,degrees,100,velocityUnits::pct);
      wait(.2,sec);
      Flywheel.stop();
      //This is repeated until all three disks are shot
      //Then reverse to down the field
      Drivetrain.driveFor(24,inches,80,velocityUnits::pct);
      //Turn to face line of three disks
      turnPTo(-135);
      turnPTo(-135);
      //Turn on intake driving almost all the way towards the disks
      Intake.spin(forward,100,percent);
      Drivetrain.driveFor(15,inches,70,velocityUnits::pct);
      wait(.5,sec);
      //turnPTo(-135);
      //slow down
      Drivetrain.driveFor(25,inches,40,velocityUnits::pct);

      //Driving forward a few inches then stopping 
      //Ensures the disks are well contained and don't get messed up
      //Ocasionally only picks up two instead of three disks
      turnPTo(-135);
      //Turn again halfway through to adjust for friction
      Drivetrain.driveFor(8,inches,50,velocityUnits::pct);
      wait(.5,sec);
      Drivetrain.driveFor(5,inches,50,velocityUnits::pct);
      wait(.3,sec);
      //Once disks are intaked turn to face blue high goal
      turnPTo(-44.5);
      //Repeat flywheel spin up wait and shoot
      Flywheel.spin(forward,11,volt);
      wait(3.4,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      //Intake.stop();
      wait(1.5,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(1.5,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(1,sec);
      Flywheel.stop();
      //Ideally this marks 6 high goal disks
      turnPTo(270);
      //Intake.spin(forward,100,velocityUnits::pct);
      Drivetrain.driveFor(5,inches,80,velocityUnits::pct);
      //Turn to face right wall and drive a bit towards it
      //Intake is turned on in case a disk isn't picked up but is in the way
      turnPTo(180);
      Drivetrain.driveFor(52,inches,70,velocityUnits::pct);
      turnPTo(270);
      //Drive towards back wall then turn towards right again
      Intake.stop();
      //After stopping intake drive in front of back roller
      Drivetrain.driveFor(47,inches,60,velocityUnits::pct);
      turnPTo(180);
      Roller.spin(forward,100,velocityUnits::pct);
      Drivetrain.drive(forward,80,velocityUnits::pct);
      wait(.6,sec);
      Drivetrain.stop();
      Drivetrain.driveFor(-1.5,inches,40,velocityUnits::pct);
      //After scoring roller turn so the back of the robot as facing the disk
      turnPTo(304+180);
      //Drive towards Right wall roller but backwards
      //This will ensure we don't drive over any disk
      Drivetrain.driveFor(-15,inches,40,velocityUnits::pct);
      turnPTo(270);
      wait(.2,sec);
      //Spin back to face the roller and wait to stop sliding
      Roller.spin(forward,100,velocityUnits::pct);
      Drivetrain.drive(forward,80,velocityUnits::pct);
      wait(.75,sec);
      Roller.stop();
      //Score the roller same as before
      Drivetrain.driveFor(-7,inches,80,velocityUnits::pct);
      //Drive backwards and then to the nearest 4 intersecting tile corner
      turnPTo(0);
      Drivetrain.driveFor(-3,inches,80,velocityUnits::pct);
      turnPTo(45);
      Drivetrain.driveFor(3,inches,80,velocityUnits::pct);
      //Facing the bigger side of the field ensure close to 3/4 tiles are touching the wheels
      //Launch pneumatics
      Pneumatic.set(true);
      Pneumatic2.set(true);
      wait(1,sec);
      



    //..........Prepare for User Control..........//
  }

  if (skills4) {

    //..........Starting..........//

    //..........Prepare for User Control..........//
  }

  if (skills5) {

    //..........Starting..........//
    //Same as skills3 or skillsnew except after shooting 6 disks
    //Turn to face field then launch pnuematics
    Roller.spin(forward,100,percent);
    Drivetrain.drive(forward,100,velocityUnits::pct);
    wait(.45,sec);
    Roller.stop();
    Drivetrain.driveFor(-1.3,inches,80,velocityUnits::pct);
    Intake.spin(forward,100,velocityUnits::pct);
    turnPTo(114);
    turnPTo(114);
    Drivetrain.driveFor(8.5,inches,55,velocityUnits::pct);
    wait(.5,sec);
    Drivetrain.driveFor(7,inches,40,velocityUnits::pct);
    wait(.75,sec);
    Drivetrain.driveFor(11,inches,65,velocityUnits::pct);
    turnPTo(90);
    turnPTo(90);
    Roller.spin(forward,100,percent);

    Drivetrain.drive(forward,50,velocityUnits::pct);
    wait(.7,sec);
    Roller.stop();
    Intake.stop();
    Drivetrain.driveFor(-1,inches,70,velocityUnits::pct);
    turnPTo(0.4);
    Drivetrain.driveFor(-36,inches,90,velocityUnits::pct);
    turnPTo(0.4);
    Flywheel.spin(forward,11,volt);
    wait(4,sec);
    Indexer.spinFor(forward,360,degrees,100,velocityUnits::pct);
    wait(1.6,sec);
    Indexer.spinFor(forward,360,degrees,100,velocityUnits::pct);
    wait(1.6,sec);
    Indexer.spinFor(forward,360,degrees,100,velocityUnits::pct);
    wait(.2,sec);
    Flywheel.stop();



    Drivetrain.driveFor(24,inches,80,velocityUnits::pct);
    turnPTo(-135);
    turnPTo(-135);


    Intake.spin(forward,100,percent);
    Drivetrain.driveFor(17,inches,70,velocityUnits::pct);
    wait(.5,sec);
    turnPTo(-135);
    turnPTo(-135);
    Drivetrain.driveFor(6,inches,50,velocityUnits::pct);
    wait(.5,sec);
    Drivetrain.driveFor(9,inches,70,velocityUnits::pct);
    wait(.3,sec);
    Drivetrain.driveFor(5,inches,50,velocityUnits::pct);
    wait(.5,sec);
    Drivetrain.driveFor(8,inches,70,velocityUnits::pct);
    wait(.3,sec);
    Drivetrain.driveFor(4,inches,50,velocityUnits::pct);
    wait(.5,sec);
    turnPTo(-135-90+180);
    Flywheel.spin(forward,11,volt);
    wait(3.5,sec);
    Indexer.spinFor(360,deg,100,velocityUnits::pct);
    wait(1.6,sec);
    Indexer.spinFor(360,deg,100,velocityUnits::pct);
    wait(1.6,sec);
    Indexer.spinFor(360,deg,100,velocityUnits::pct);
    wait(1,sec);
    Flywheel.stop();
    Intake.stop();
    Drivetrain.driveFor(-5,inches,100,velocityUnits::pct);
    Pneumatic.set(true);
    Pneumatic2.set(true);


    //..........Prepare for User Control..........//
  }

  if (skills6) {

    //..........Starting..........//
   
    //..........Prepare for User Control..........//
  }

  if (skills7) {
    //..........Starting..........//
    
      Drivetrain.driveFor(20,inches,80,velocityUnits::pct);
      turnPTo(90);
      Roller.spin(reverse,100,velocityUnits::pct);
      Drivetrain.drive(forward,90,velocityUnits::pct);
      wait(.5,sec);
      Roller.stop();
      Drivetrain.stop();
      Drivetrain.driveFor(-2,inches,90,velocityUnits::pct);
      turnPTo(90.5);
      Flywheel.spin(forward,12,volt);
      Drivetrain.driveFor(-5,inches,60,velocityUnits::pct);
      wait(5.3,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(1.35,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(2,sec);
      Flywheel.stop();
  
    //.......Prepare for User Control...............//
  }

  if (skills8) {

    //..........Starting..........//
          Roller.spin(reverse,100,velocityUnits::pct);
      Drivetrain.drive(forward,90,velocityUnits::pct);
      wait(.5,sec);
      Roller.stop();
      Drivetrain.stop();
      Drivetrain.driveFor(-4,inches,70,velocityUnits::pct);
      turnPTo(-7);
      Flywheel.spin(forward,12,volt);
      Drivetrain.driveFor(-1,inches,60,velocityUnits::pct);
      wait(5.3,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(1.4,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(2,sec);
      Flywheel.stop();

    //..........Prepare for User Control..........//
  }
}

  //...............END OF CODE...............//


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*                                                                           */
/*---------------------------------------------------------------------------*/
bool halfspeed = false;
bool soloControl = false;
//void solo() {soloControl = !soloControl;}

//Intake reverse function
void IntakeRev(){
  Intake.spin(reverse,30,percent);
}
//Intake forward function
void IntakeFor(){
  Intake.spin(forward,100,percent);
}
void halfspeedcontrol() { halfspeed = !halfspeed; }
bool clickState1 = true;
bool motorState1 = false;
bool clickState2 = true;
bool motorState2 = false;
double speedVar = Flywheel.voltage(volt);
void FlywheelSpeedDown() {
  speedVar = Flywheel.voltage();
  Flywheel.spin(forward,speedVar-.5,volt);
  speedVar=Flywheel.voltage();

}
void FlywheelSpeedUp() {
  speedVar = Flywheel.voltage();
  Flywheel.spin(forward,speedVar+.5,volt);
  speedVar=Flywheel.voltage();


}
void pneumatic(){
  Pneumatic.set(true);
  Pneumatic2.set(true);
}
void pneumatic2(){
  Pneumatic.set(false);
  Pneumatic2.set(false);
}


void usercontrol(void) {

  int threshold = 20, ChasLfVar = 0, ChasRtVar = 0;

  //Controller1.ButtonA.pressed(halfspeedcontrol);

  Controller2.ButtonDown.pressed(FlywheelSpeedDown);
  Controller2.ButtonUp.pressed(FlywheelSpeedUp);
  Controller2.ButtonR2.pressed(IntakeRev);
  Controller1.ButtonL1.pressed(IntakeFor);
  Controller1.Screen.clearScreen();

  while (1) {


    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    //Set stopping for motors
    Flywheel.setStopping(coast);
    Intake.setStopping(coast);


    //Test Button for Autonmous
    //Empty for compeititon
    if(Controller1.ButtonX.pressing()){
      //right long
      /*
      Drivetrain.driveFor(20,inches,80,velocityUnits::pct);
      turnPTo(90);
      Roller.spin(reverse,100,velocityUnits::pct);
      Drivetrain.drive(forward,90,velocityUnits::pct);
      wait(.5,sec);
      Roller.stop();
      Drivetrain.stop();
      Drivetrain.driveFor(-2,inches,90,velocityUnits::pct);
      turnPTo(90.5);
      Flywheel.spin(forward,12,volt);
      Drivetrain.driveFor(-5,inches,60,velocityUnits::pct);
      wait(5.3,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(1.35,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(2,sec);
      Flywheel.stop();
      */
          //  Drivetrain.driveFor(20,inches,80,velocityUnits::pct);
      //turnPTo(90);
      /*
      Roller.spin(reverse,100,velocityUnits::pct);
      Drivetrain.drive(forward,90,velocityUnits::pct);
      wait(.5,sec);
      Roller.stop();
      Drivetrain.stop();
      Drivetrain.driveFor(-4,inches,70,velocityUnits::pct);
      turnPTo(-7);
      Flywheel.spin(forward,12,volt);
      Drivetrain.driveFor(-1,inches,60,velocityUnits::pct);
      wait(5.3,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(1.4,sec);
      Indexer.spinFor(360,deg,100,velocityUnits::pct);
      wait(2,sec);
      Flywheel.stop();
*/

    }
    // Tank Drivetrain //

    // Left Side Chassis
    if (abs(Controller1.Axis3.position(percentUnits::pct)) > threshold) 
    {
      ChasLfVar = Controller1.Axis3.position(percentUnits::pct);
    } else {
      ChasLfVar = 0;
      LeftDriveSmart.stop(brakeType::brake);
    }

    // Right Side Chassis
    if (abs(Controller1.Axis2.position(percentUnits::pct)) > threshold) 
    {
      ChasRtVar = Controller1.Axis2.position(percentUnits::pct);
    } else {
      ChasRtVar = 0;
      RightDriveSmart.stop(brakeType::brake);
    }
    // halfspeed control //
    //Allows speed to be switch to 50 percent for drivetrain
    //Currently button is disabled above
    if (halfspeed == true) {
      LeftDriveSmart.spin(directionType::fwd, ChasLfVar * .50,percentUnits::pct);
      RightDriveSmart.spin(directionType::fwd, ChasRtVar * .50, percentUnits::pct);
    } else {
      LeftDriveSmart.spin(directionType::fwd, ChasLfVar, percentUnits::pct);
      RightDriveSmart.spin(directionType::fwd, ChasRtVar, percentUnits::pct);
    }
    if (Controller1.ButtonR1.pressing()) {
      Indexer.spinFor(forward,360,degrees,75,velocityUnits::pct);
    }
    if(Controller1.ButtonUp.pressing()) {
      Flywheel.spin(forward,12,volt);
      speedVar=Flywheel.voltage(volt);
    }
    if(Controller1.ButtonDown.pressing()){
      Flywheel.spin(forward,10.5,volt);
      speedVar=Flywheel.voltage(volt);
    }
    if(Controller1.ButtonLeft.pressing()){
      Flywheel.stop();
      tbhSwitch=false;
    }
    if(Controller1.ButtonRight.pressing()){
      Flywheel.spin(forward);
    }
    if(!Controller1.ButtonA.pressing()&& !Controller1.ButtonB.pressing()){
      pneumatic2();
    }
    if(Controller1.ButtonA.pressing()&&Controller1.ButtonB.pressing()){
      pneumatic();
    }
    //Intake stop control
    if(Controller1.ButtonL2.pressing()){
      Intake.stop();
    }
    //Roller Control
    //Allows roller to be on one button
    if(Controller1.ButtonR2.pressing()&&clickState2){
      motorState2=!motorState2;
      clickState2=false;
    }
    if(!Controller1.ButtonR2.pressing()) {clickState2=true;}

    if(motorState2){
      Roller.spin(forward,80,percent);
    }
    if(!motorState2){
      Roller.stop();
    }

    //Flywheel Control
    if(Controller2.ButtonL1.pressing()){
      Flywheel.spin(forward,500,rpm);
    }
    if(Controller2.ButtonL2.pressing()){
      Flywheel.spin(forward,600,rpm);
    }
    if(Controller2.ButtonR1.pressing()){
      Flywheel.spin(forward, 400,rpm);
    }
    if(Controller2.ButtonLeft.pressing()){
      Flywheel.stop();
    }
  }




    

  wait(100,msec);
}

// Main will set up the competition functions and callbacks

int main() {

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Run the pre-autonomous function
  pre_auton();

  //Set up custom tasks
  //task TrackPositionTask(trackPosition);
  // task VisionTask(visionTurn);
  task printTask(printRPM);
  // task flywheelTask(tbh);

  // Set up callbacks for autonomous and driver control periods
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Register events for button selection
  Brain.Screen.pressed(userTouchCallbackPressed);
  Brain.Screen.released(userTouchCallbackReleased);

  // Make nice background
  Brain.Screen.setFillColor(vex::color(0x404040));
  Brain.Screen.setPenColor(vex::color(0x404040));
  Brain.Screen.drawRectangle(0, 0, 480, 120);
  Brain.Screen.setFillColor(vex::color(0x808080));
  Brain.Screen.setPenColor(vex::color(0x808080));
  Brain.Screen.drawRectangle(0, 120, 480, 120);
  //For let the gods so speed me, as I love the name of honour more than I fear death
  //thou art noble; yet, I see, thy honourable metal may be wrought
  //nobilis es; verumtamen, ut video, metallum tuum honestum fieri potest
  // Initial Display
  displayButtonControls(0, false);



  // While loop to call back functions to run during competition
  while (1) {
    // Allow other tasks to run

    if (!Competition.isEnabled())
      Brain.Screen.setFont(fontType::mono40);
      Brain.Screen.setFillColor(vex::color(0xFFFFFF));
      Brain.Screen.setPenColor(vex::color(0xc11f27));
      Brain.Screen.printAt(0, 135, "For the Republic");
      this_thread::sleep_for(10);
  }
}
