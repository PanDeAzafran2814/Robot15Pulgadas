#include "vex.h"

using namespace vex;

competition Competition;
controller Control = controller(primary);
brain Brain;
// Front Pincers
motor FrontLeftPincer = motor(PORT5, ratio36_1, false);
motor FrontRightPincer = motor(PORT6, ratio36_1, true);
// Back Pincers
motor BackLeftPincer = motor(PORT15, ratio36_1, false);
motor BackRightPincer =motor(PORT16, ratio36_1, true);
// Right Wheels
motor FrontRightWheel = motor(PORT10, ratio18_1, true);
motor BackRightWheel = motor(PORT20, ratio18_1, true);
// Left Wheels
motor FrontLeftWheel = motor(PORT1, ratio18_1, false);
motor BackLeftWheel = motor(PORT11, ratio18_1, false);
// Motor Groups
motor_group LeftWheels = motor_group(FrontLeftWheel, BackLeftWheel);
motor_group RightWheels = motor_group(BackRightWheel, FrontRightWheel);
motor_group BackPincers = motor_group(BackLeftPincer, BackRightPincer);
motor_group FrontPincers = motor_group(FrontLeftPincer, FrontRightPincer);
// Buttons
bumper FrontButton = bumper(Brain.ThreeWirePort.A);
// Drivetrain & Inertial Sensor
inertial InertialSensor = inertial(PORT19);
smartdrive Drivetrain = smartdrive(LeftWheels, RightWheels, InertialSensor, 319.185806, 320, 280, mm, 1.6);

bool isReverse = false;
bool buttonPressed = false;
bool lowVelocity = false;
int count = 0;
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  InertialSensor.calibrate();
  vexcodeInit();
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

// Función cambiar velocidad
void changeVelocity(int v) {
  RightWheels.setVelocity(v, percent);
  LeftWheels.setVelocity(v, percent);
  Drivetrain.setTurnVelocity(v, percent);
  Drivetrain.setDriveVelocity(v, percent);
  RightWheels.setMaxTorque(v, percent);
  LeftWheels.setMaxTorque(v,percent);
}


// Función Multithread
int frontPincersThread() {
  
  this_thread::sleep_for(25);
  
  return 0;
}

// Instrucciones llegar avanzar al plato amarillo
void autonomousYellowGoal() {
  thread myThread = thread(frontPincersThread);
  //thread mySecondThread = thread(backPincersThread);
  Drivetrain.drive(forward);
}


// Main Autonomo
void autonomous(void) {
  changeVelocity(100);
  FrontPincers.setVelocity(100, percent);
  BackPincers.setVelocity(100, percent);
  RightWheels.resetRotation();
  LeftWheels.resetRotation();
  autonomousYellowGoal();
  
}
/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void changeMovement() {
  if(Control.ButtonDown.pressing()) {
    isReverse = true;
  }
  if(Control.ButtonUp.pressing()) {
    isReverse = false;
  }
}

void changeUserVelocity() {
  if(Control.ButtonA.pressing()) {
    lowVelocity = true;
  }
  if(Control.ButtonB.pressing()) {
    lowVelocity = false;
  }
}

float getVelocity(float v) {
  if(lowVelocity) {
    return v * 0.1;
  } else {
    return v;
  }
}

void leftMovement() {
  if(Control.Axis3.value() != 0){
    if(isReverse) {
      RightWheels.spin(reverse, getVelocity(Control.Axis3.value()), percent);
    } else {
      LeftWheels.spin(forward, getVelocity(Control.Axis3.value()), percent);
    }
  } else {
    LeftWheels.stop(hold);
  }
}

void rightMovement() {
  if(Control.Axis2.value() != 0) {
    if(isReverse) {
      LeftWheels.spin(reverse, getVelocity(Control.Axis2.value()), percent);
    } else {
      RightWheels.spin(forward, getVelocity(Control.Axis2.value()), percent);
    }
  } else {
    RightWheels.stop(hold);
  }
}

void frontPincersMovement() {
  if(isReverse) {
      if(Control.ButtonR1.pressing()) {
        BackPincers.spin(reverse);
    } else if(Control.ButtonR2.pressing()) {
        BackPincers.spin(forward);
    } else {
        BackPincers.stop(hold);
    }
  } else {
      if(Control.ButtonR1.pressing()){
        FrontPincers.spin(forward);
    } else if(Control.ButtonR2.pressing()) {
        FrontPincers.spin(reverse);
    } else {
        FrontPincers.stop(hold);
    }
  }
}

void backPincersMovement() {
  if(isReverse) {
    if(Control.ButtonL1.pressing()) {
      FrontPincers.spin(forward);
    } else if(Control.ButtonL2.pressing()) {
      FrontPincers.spin(reverse);
    } else {
      FrontPincers.stop(hold);
    }
  } else {
      if(Control.ButtonL1.pressing()) {
      BackPincers.spin(reverse);
    } else if(Control.ButtonL2.pressing()) {
      BackPincers.spin(forward);
    } else {
      BackPincers.stop(hold);
    }
  }
}

void usercontrol(void) {
  LeftWheels.setVelocity(100, percent);
  RightWheels.setVelocity(100, percent);
  FrontPincers.setVelocity(80, percent);
  BackPincers.setVelocity(80, percent);
  FrontPincers.setMaxTorque(100, percent);
  BackPincers.setMaxTorque(100, percent);
  while (1) {
    changeMovement();
    changeUserVelocity();
    leftMovement();
    rightMovement();
    frontPincersMovement();
    backPincersMovement();
    wait(20, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
