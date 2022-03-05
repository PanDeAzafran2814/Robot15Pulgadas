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
digital_out FrontValve = digital_out(Brain.ThreeWirePort.A);
// Drivetrain & Inertial Sensor
inertial InertialSensor = inertial(PORT18);
smartdrive Drivetrain = smartdrive(LeftWheels, RightWheels, InertialSensor, 299.24, 292.1, 279.4, mm, 1.2);

bool isReverse = false;
bool lowVelocity = false;
int bandaControl = 0;

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

// Configuraciones iniciales del robot
void initConfig(int v, int t) {
  RightWheels.setMaxTorque(t, percent);
  LeftWheels.setMaxTorque(t,percent);
  FrontPincers.setVelocity(v, percent);
  BackPincers.setVelocity(v, percent);
  RightWheels.resetRotation();
  LeftWheels.resetRotation();
}

// Función cambiar velocidad
void changeVelocity(int v) {
  RightWheels.setVelocity(v, percent);
  LeftWheels.setVelocity(v, percent);
  Drivetrain.setTurnVelocity(v, percent);
  Drivetrain.setDriveVelocity(v, percent);
}


void moveArmsThread() {
  FrontPincers.spinFor(reverse, 400, degrees);
}

void RafaAutonomo() {
  initConfig(100, 100);
  changeVelocity(100);
  thread handsThread = thread(moveArmsThread);
  Drivetrain.driveFor(forward, 41, inches);
  wait(150, msec);
  changeVelocity(80);
  FrontPincers.spinFor(forward, 180, degrees);
  Drivetrain.driveFor(reverse, 30, inches);
  Drivetrain.turnFor(left, 90, degrees);
  BackPincers.spinFor(forward, 450, degrees);
  Drivetrain.driveFor(reverse, 13, inches);
  BackPincers.spinFor(reverse, 180, degrees);
}

// Main Autonomo
void autonomous(void) {
  
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

void valveManagement(){
  if(Control.ButtonLeft.pressing()) {
    FrontValve.set(0);
  }
  if(Control.ButtonRight.pressing()) {
    FrontValve.set(1);
  }
}

void manageBandaControl(){
  if(bandaControl == 0) {
    Banda.stop(hold);    
  } else if(bandaControl == 1) {
    Banda.spin(forward, 85, percent);
  } else if(bandaControl == 2) {
    Banda.spin(reverse, 85, percent);
  }
}

void manageBanda() {
  if(Control.ButtonA.pressing()) {
    bandaControl = 1;
  }
  else if(Control.ButtonB.pressing()) {
    bandaControl = 2;
  } else if(Control.ButtonX.pressing()){
    bandaControl = 0;
  }
}

void leftMovement() {
  if(Control.Axis3.value() != 0){
    if(isReverse) {
      RightWheels.spin(reverse, Control.Axis3.value(), percent);
    } else {
      LeftWheels.spin(forward, Control.Axis3.value(), percent);
    }
  } else {
    LeftWheels.stop(hold);
  }
}

void rightMovement() {
  if(Control.Axis2.value() != 0) {
    if(isReverse) {
      LeftWheels.spin(reverse, Control.Axis2.value(), percent);
    } else {
      RightWheels.spin(forward, Control.Axis2.value(), percent);
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
  FrontPincers.setVelocity(100, percent);
  BackPincers.setVelocity(100, percent);
  FrontPincers.setMaxTorque(100, percent);
  BackPincers.setMaxTorque(100, percent);
  while (1) {
    changeMovement();
    manageBandaControl();
    manageBanda();
    leftMovement();
    rightMovement();
    frontPincersMovement();
    backPincersMovement();
    valveManagement();
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
