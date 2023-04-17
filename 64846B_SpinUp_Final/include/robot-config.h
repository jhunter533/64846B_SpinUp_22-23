using namespace vex;

extern brain Brain;

// VEXcode devices


// VEXcode devices

extern triport ThreeWire;
extern controller Controller1;
extern motor FrontLeft;
extern motor BackLeft;
extern motor FrontRight;
extern motor BackRight;
extern controller Controller2;
extern inertial TurnGyroSmart;
extern motor Indexer;
extern motor Intake;
extern motor Flywheel;
extern encoder Back;
extern encoder Right;
extern digital_out Pneumatic;
extern smartdrive Drivetrain;
extern motor_group RightDriveSmart;
extern motor_group LeftDriveSmart;
extern digital_out Pneumatic2;
extern motor Roller;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );