#include "vex.h"

using namespace vex;

competition Competition;
controller Control = controller(primary);
brain Brain;
// Up Pincer
motor Banda = motor(PORT17, ratio18_1, false);
// Front Pincers
motor FrontLeftPincer = motor(PORT5, ratio36_1, true);
motor FrontRightPincer = motor(PORT6, ratio36_1, false);
// Back Pincers
motor BackLeftPincer = motor(PORT15, ratio36_1, false);
motor BackRightPincer =motor(PORT16, ratio36_1, true);
// Right Wheels
motor FrontRightWheel = motor(PORT10, ratio18_1, true);
motor MiddleRightWheel = motor(PORT19, ratio18_1, false);
motor BackRightWheel = motor(PORT20, ratio18_1, true);
motor UpRightWheel = motor(PORT9, ratio18_1, false);
// Left Wheels
motor FrontLeftWheel = motor(PORT1, ratio18_1, false);
motor MiddleLeftWheel = motor(PORT12, ratio18_1, true);
motor BackLeftWheel = motor(PORT11, ratio18_1, false);
motor UpLeftWheel = motor(PORT2, ratio18_1, true);
// Motor Groups
motor_group LeftWheels = motor_group(FrontLeftWheel, MiddleLeftWheel, BackLeftWheel, UpLeftWheel);
motor_group RightWheels = motor_group(BackRightWheel, MiddleRightWheel, FrontRightWheel, UpRightWheel);
motor_group BackPincers = motor_group(BackLeftPincer, BackRightPincer);
motor_group FrontPincers = motor_group(FrontLeftPincer, FrontRightPincer);
// Buttons
digital_out BackValve = digital_out(Brain.ThreeWirePort.A);
// Drivetrain & Inertial Sensor
inertial InertialSensor = inertial(PORT18);
distance DistanceSensor = distance(PORT3);
smartdrive Drivetrain = smartdrive(LeftWheels, RightWheels, InertialSensor, 299.24, 292.1, 279.4, mm, 1.2);

bool isReverse = false;
bool lowVelocity = false;
int bandaControl = 0;