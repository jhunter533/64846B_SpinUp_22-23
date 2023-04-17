#ifndef PID
#define PID

typedef struct _pid{
  double integral;
  double derivative;
  double error;
  double prevError;
  double powerDrive;
  double kP;
  double kD;
  double kI;
  double threshold;
  double iBound;
  double maxSpeed = 10;
  double powerDriveCon;
} PID_;

extern PID_ turnPID;
extern PID_ driveX;
extern PID_ driveY;
extern PID_ drivePID;
void driveToNS();
void driveTo();
void turnTo();
void driveToX();
void driveToY();



#endif