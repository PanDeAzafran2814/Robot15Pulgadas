#include "vex.h"

using namespace vex;

competition Competition;
controller Control = controller(primary);
brain Brain;
// Up Pincer
motor UpPincer = motor(PORT17, ratio18_1, false);
// Front Pincers
motor FrontLeftPincer = motor(PORT5, ratio36_1, false);
motor FrontRightPincer = motor(PORT6, ratio36_1, true);
// Back Pincers
motor BackLeftPincer = motor(PORT15, ratio36_1, false);
motor BackRightPincer =motor(PORT16, ratio36_1, true);
// Right Wheels
motor FrontRightWheel = motor(PORT10, ratio18_1, true);
motor MiddleRightWheel = motor(PORT19, ratio18_1, false);
motor BackRightWheel = motor(PORT20, ratio18_1, true);
// Left Wheels
motor FrontLeftWheel = motor(PORT1, ratio18_1, false);
motor MiddleLeftWheel = motor(PORT12, ratio18_1, true);
motor BackLeftWheel = motor(PORT11, ratio18_1, false);
// Motor Groups
motor_group LeftWheels = motor_group(FrontLeftWheel, MiddleLeftWheel, BackLeftWheel);
motor_group RightWheels = motor_group(BackRightWheel, MiddleRightWheel, FrontRightWheel);
motor_group BackPincers = motor_group(BackLeftPincer, BackRightPincer);
motor_group FrontPincers = motor_group(FrontLeftPincer, FrontRightPincer);
// Buttons
bumper FrontButton = bumper(Brain.ThreeWirePort.A);
// Drivetrain & Inertial Sensor
inertial InertialSensor = inertial(PORT18);
smartdrive Drivetrain = smartdrive(LeftWheels, RightWheels, InertialSensor, 299.24, 292.1, 279.4, mm, 1.2);

bool isReverse = false;
bool lowVelocity = false;

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

// Función Multithread bajar los brazos
int frontPincersThread() {
  FrontPincers.spinFor(reverse, 750, degrees);
  this_thread::sleep_for(25);
  
  return 0;
}

// Instrucciones llegar avanzar al plato amarillo
void autonomousYellowGoal() {
  thread myThread = thread(frontPincersThread);
  Drivetrain.drive(forward);
}

// Instrucciones regresar con el plato amarillo
void autonomousBackYellowGoal() {
  while(1) {
    if(FrontButton.pressing() || RightWheels.rotation(degrees) > 2300) {
      Drivetrain.stop();
      wait(250, msec);
      FrontPincers.spinFor(forward, 300, degrees);
      UpPincer.spinFor(forward, 450, degrees);
      changeVelocity(50);
      Drivetrain.driveFor(reverse, 55, inches);
      break;
    }
  }
}

void autonomousTeamGoal() {
  Drivetrain.turnFor(left, 40, degrees);
  BackPincers.spinFor(forward, 750, degrees);
  Drivetrain.driveFor(reverse, 10, inches);
  BackPincers.spinFor(reverse, 300, degrees);
}

// Main Autonomo
void autonomous(void) {
  initConfig(100, 100);
  changeVelocity(100);
  autonomousYellowGoal();
  autonomousBackYellowGoal();
  autonomousTeamGoal();
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

void manageUpPincer() {
  if(Control.ButtonA.pressing()) {
    UpPincer.spin(forward, 100, percent);
  }
  else if(Control.ButtonB.pressing()) {
    UpPincer.spin(reverse, 100, percent);
  } else {
    UpPincer.stop(hold);
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
  FrontPincers.setVelocity(100, percent);
  BackPincers.setVelocity(100, percent);
  FrontPincers.setMaxTorque(100, percent);
  BackPincers.setMaxTorque(100, percent);
  while (1) {
    changeMovement();
    manageUpPincer();
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
