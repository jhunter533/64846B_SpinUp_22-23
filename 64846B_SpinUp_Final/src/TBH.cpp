#include "TBH.h"
#include "GlobalVariables.h"
#include "Odometry.h"
#include "vex.h"
#include "Conversion.h"
#include "PID.h"
#include "oldPID.h"
#include "Graphics.h"
bool tbhSwitch = true;
double e;
double output;
double gain=.0005;
double prevE=0;
double tbhV;
double targetRPM;

void tbhInput(double tarRPM){
targetRPM=tarRPM;
tbhSwitch=true;
}

int tbh(){
  while(true){
    if(tbhSwitch==true){

      
      e=targetRPM-Flywheel.velocity(rpm);
      output+=gain*e;
      printf(" %.5f", e);
      printf(",  %.5f \n", output);
  
      
      if(output>1){
        output=1;
      }
      if(output<0){
        output=0;
      }
      if(signbit(e)!=signbit(prevE)){
        output=.5*(output+tbhV);
        tbhV=output;
        prevE=e;
        
      }

    }
    else {
      
      output=0;
    }
    //Flywheel.spin(forward,output*600,rpm);
    task::sleep(20);
  }
  return 1;
}
