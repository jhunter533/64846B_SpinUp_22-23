#include "Conversion.h"
#include "GlobalVariables.h"
#include "vex.h"
#include "PID.h"
#include "Odometry.h"
double angleTarget;

double ConvertToDeg(double radians) {
  return (radians *180/M_PI);
}
double ConvertToRadians(double degree) {
  return (degree*M_PI/180);
}
void shortAngle() {
  double Ang1=ConvertToDeg(atan2(yVector,xVector))-ConvertToDeg(absoluteAngle);
  double fastAngle = Ang1-ConvertToDeg(absoluteAngle)-360;
  double fastAngle2=Ang1-ConvertToDeg(absoluteAngle)+360;
  if(fabs(Ang1)<fabs(fastAngle) && fabs(Ang1) <fabs(fastAngle2)){
    angleTarget=ConvertToRadians(Ang1);
          printf("1 \n");
  }
  if(fabs(Ang1)>fabs(fastAngle)&&fabs(fastAngle) < fabs(fastAngle2)){
    angleTarget=ConvertToRadians(fastAngle);
      printf("2 \n");
  }
  if(fabs(Ang1)> fabs(fastAngle2) &&fabs(fastAngle2)<fabs(fastAngle2)){
    angleTarget=ConvertToRadians(fastAngle2);
          printf("3 \n");
  }
  printf("ang1 %.5f\n", Ang1);
  printf("fastang1 %.5f\n", fastAngle);
  printf("fastang2 %.5f\n", fastAngle2);
}
/*

void ShortAngle() {
  double sigma = (targetFacA-absoluteAngle);
  double beta = (targetFacA-absoluteAngle)+360;
  //double alpha = (360 - targetFacA) - absoluteAngle - 360;

  if (fabs(sigma)<fabs(beta) ){
    turnPID.error = sigma;
  }
  if (fabs(beta)<fabs(sigma)) {
    turnPID.error = beta;
  }

}
*/