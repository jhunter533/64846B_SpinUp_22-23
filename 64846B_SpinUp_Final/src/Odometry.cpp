//includes for all files//
#include "GlobalVariables.h"
#include "Odometry.h"
#include "vex.h"
#include "Conversion.h"
#include "PID.h"
#include "oldPID.h"
#include "Graphics.h"
//includes for all files//

//declare all global variables//

//declare motor power variables
double FrontLeftPower;
double BackLeftPower;
double FrontRightPower;
double BackRightPower;

//declare starting values
//start angle does nothing unless absolute angle is set to it
double startAngle = M_PI/2;
double startPosX=(2*360)/(M_PI*3.25);
double startPosY = (2*360)/(M_PI*3.25);

//declare distance from tracking wheels to tracking center
double sDistanceInput=5.6;
double rDistanceInput=4.7;
//Convert the distance in inches to degrees
double sTrackDistance = (sDistanceInput*360)/(M_PI*3.25);
double rTrackDistance = (rDistanceInput*360)/(M_PI*3.25);


//diameter of tracking wheels
double wheelDiameter = 3.25;

//declare the variable to store previous encoder distances
float prevR = 0;
float prevS = 0;
//double absoluteAngle=startAngle;
//declare global tracking variables
double absoluteAngle = ConvertToRadians(-TurnGyroSmart.rotation());
double prevTheta = absoluteAngle;
double totalDeltaDistR = 0;
double yPosGlobal = startPosY;
double xPosGlobal = startPosX;
double deltaTheta = 0;
double halfAngle = deltaTheta/2;
double avgTheta = absoluteAngle-halfAngle;

// the total change in position overtime
double deltaXGlobal = 0;
double deltaYGlobal = 0;

//the change in position from previous iteration to current
double deltaXLocal = 0;
double deltaYLocal = 0;
double r = 0;
double r2 = 0;


double xTargetLocation = xPosGlobal;
double yTargetLocation = yPosGlobal;

double xVector = 0;
double yVector = 0;

double deltaR = 0;
double deltaS = 0;

int trackPosition() {


while(1) {

  //change in encoder value between iterations
  deltaR = (Right.rotation(deg)-prevR);
  deltaS = (Back.rotation(deg)- prevS);

  //previous encoder value
  prevR = Right.rotation(deg);
  prevS = Back.rotation(deg);

  //total change in encoder over all time
  totalDeltaDistR += deltaR;

  //the current heading of the robot converted to radians
  absoluteAngle=ConvertToRadians(-TurnGyroSmart.rotation(deg));


  //the change in theta between iterations
  deltaTheta = absoluteAngle - prevTheta;
  //previous angle
  prevTheta = absoluteAngle;

  //if the robot's change in theta exists then we moved in 2 axis
  if ((deltaTheta !=0)) {
  halfAngle = deltaTheta/2;
  r = deltaR/deltaTheta;
  r2 = deltaS/deltaTheta;
  deltaXLocal = 2*sin(halfAngle) *(r2-sTrackDistance);
  deltaYLocal = 2*sin(halfAngle) * (r-rTrackDistance);

  //if the angle didn't change then we only moved in one direction
  } else {

  deltaXLocal = (deltaS);
  deltaYLocal = (deltaR);

  halfAngle = 0;
  }
avgTheta=absoluteAngle-halfAngle;
  //the change in theta of the coordinates
  deltaYGlobal = (deltaXLocal) *cos(avgTheta) + (deltaYLocal) * sin(avgTheta);
  deltaXGlobal = (deltaYLocal) * cos(avgTheta) - (deltaXLocal) * sin(avgTheta);

  //adds the change to the original coordinates yielding new coordinates
  xPosGlobal += (deltaXGlobal);
  yPosGlobal += (deltaYGlobal);

  //draws coordinate grid on brain
  //draw();

//repeats the iteration every 10 miliseconds
  task::sleep(10);

  }
return 1;
}


void driveToP(double xTarget, double yTarget, double alpha,double beta) {
  //the x and y target converted to degrees traveled
  xTargetLocation = (xTarget*360)/(M_PI*wheelDiameter);
  yTargetLocation = (yTarget*360)/(M_PI*wheelDiameter);

  //the difference between targets - current
  xVector = xTargetLocation - xPosGlobal;
  yVector = yTargetLocation - yPosGlobal;

  //while loop of the vector needed to travel and the angle left to turn
  //once below threshold should exit
  shortAngle();

  while(sqrt(xVector*xVector+yVector*yVector)>26 || (atan2(yVector,xVector)-absoluteAngle) >=ConvertToRadians(5) ) {
    xVector = xTargetLocation - xPosGlobal;
    yVector = yTargetLocation - yPosGlobal;
    driveToNS();
    turnTo();

    //the drive power for each motor
    //combines the drive and turn using alpha to determine the ratio
    FrontLeftPower = beta*(alpha*(drivePID.powerDrive) -(1-alpha)*turnPID.powerDriveCon);
    BackLeftPower = beta*(alpha*drivePID.powerDrive-(1-alpha)*turnPID.powerDriveCon);
    FrontRightPower = beta*(alpha*drivePID.powerDrive + (1-alpha)*turnPID.powerDriveCon);
    BackRightPower = beta*(alpha*drivePID.powerDrive +(1-alpha)*turnPID.powerDriveCon);
   
    /*
    //an attempt at reversing if target is passed

    if ((xTargetLocation<=xPosGlobal-1&&xTargetLocation<=xPosGlobal+1) || (yTargetLocation<yPosGlobal-1 && yTargetLocation<yPosGlobal+1)){
    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);
      if(sqrt(xVector*xVector+yVector*yVector)<=2 ||(atan2(yVector,-xVector)-absoluteAngle-M_PI/2)<=ConvertToDeg(2)){
      break;
      }
    } else if ((xTarget>=xPosGlobal+1 && xTarget>=xPosGlobal-1) || (yTargetLocation>yPosGlobal+1 && yTargetLocation>yPosGlobal-1)) {
    FrontLeft.spin(forward,-FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,-FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,-BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,-BackRightPower,voltageUnits::volt);
        if(sqrt(xVector*xVector+yVector*yVector)<=2 &&((atan2(yVector,-xVector)-absoluteAngle-M_PI/2)<=ConvertToDeg(2))){
      break;
      }
    } else {
    break;
      }
    */

    //sends velocity in volts to each motor to actually drive
    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);

    //print data to console for checking
    
    printf( "      \n");
    printf( "xPos %.5f\n", xPosGlobal);
    printf( "yPos %.5f\n", yPosGlobal);
    printf( "sqrt %.5f\n", sqrt(xVector*xVector+yVector*yVector));

    printf( "atan2 %.5f\n", atan2(yVector,xVector)-absoluteAngle);
    printf( "targethe %.5f\n", atan2(yVector,xVector));
    printf( "ytarget %.5f\n", yTargetLocation);
    printf( "xtarget %.5f\n", xTargetLocation);
    printf( "FLP %.5f\n", FrontLeftPower);
    printf( "currtheta %.5f\n",absoluteAngle);
    printf( "turn pd %.5f\n", turnPID.powerDriveCon);
    printf( "drivepid pd %.5f\n", drivePID.powerDrive);
    printf( "New Iteration00 \n");
    printf( "      \n");

    //double ensurance to exit loop if the target is close enough
    /*
    if (fabs(xTarget-xPosGlobal) <=20 && fabs(yTarget-yPosGlobal) <=20){
      break;

    }*/
    //repeat loop every 15 miliseconds

    if (fabs(xVector) <=15 && fabs(yVector) <= 15 ){//&& atan2(yVector,-xVector)-absoluteAngle-M_PI/2<=ConvertToRadians(4)){
      printf("exit \n");
      break;
    }
    task::sleep(15);
  }

//once while loop is not true stop motors
  FrontLeft.stop();
  FrontRight.stop();
  BackRight.stop();
  BackLeft.stop();
  
}

void DriveX(double xTargetx){
  xTargetLocation = (xTargetx*360)/(M_PI*wheelDiameter);
  xVector = xTargetLocation - xPosGlobal;
  while(xVector>10) {
    xVector = xTargetLocation-xPosGlobal;
    driveToX();
    FrontLeftPower = driveX.powerDrive;
    BackLeftPower = driveX.powerDrive;
    FrontRightPower = driveX.powerDrive;
    BackRightPower = driveX.powerDrive;
    if ((xVector>0)){
      FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
      FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
      BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
      BackRight.spin(forward,BackRightPower,voltageUnits::volt);
      printf( "forward \n");
      printf( "flPower %.5f\n", FrontLeftPower);
    } else if ((xVector<0) ) {
      FrontLeft.spin(forward,-FrontLeftPower,voltageUnits::volt);
      FrontRight.spin(forward,-FrontRightPower,voltageUnits::volt);
      BackLeft.spin(forward,-BackLeftPower,voltageUnits::volt);
      BackRight.spin(forward,-BackRightPower,voltageUnits::volt);
      printf( "reverse \n");
      printf( "flPower %.5f\n", FrontLeftPower);
    } else {
      FrontLeft.stop();
      FrontRight.stop();
      BackLeft.stop();
      BackRight.stop();
      printf( "break \n");
      break;
    }
  }
}
void DriveY(double yTargety){
  yTargetLocation = (yTargety*360)/(M_PI*wheelDiameter);
  yVector = yTargetLocation - yPosGlobal;
  while(xVector>10) {
    xVector = yTargetLocation-yPosGlobal;
    driveToY();
    FrontLeftPower = driveY.powerDrive;
    BackLeftPower = driveY.powerDrive;
    FrontRightPower = driveY.powerDrive;
    BackRightPower = driveY.powerDrive;
    if ((yVector>0)){
      FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
      FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
      BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
      BackRight.spin(forward,BackRightPower,voltageUnits::volt);
      printf( "forward \n");
      printf( "flPower %.5f\n", FrontLeftPower);
    } else if ((xVector<0) ) {
      FrontLeft.spin(forward,-FrontLeftPower,voltageUnits::volt);
      FrontRight.spin(forward,-FrontRightPower,voltageUnits::volt);
      BackLeft.spin(forward,-BackLeftPower,voltageUnits::volt);
      BackRight.spin(forward,-BackRightPower,voltageUnits::volt);
      printf( "reverse \n");
      printf( "flPower %.5f\n", FrontLeftPower);
    } else {
      FrontLeft.stop();
      FrontRight.stop();
      BackLeft.stop();
      BackRight.stop();
      printf( "break \n");
      break;
    }
  }
}