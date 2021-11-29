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

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  vexcodeInit();
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void changeVelocity(int v) {
  RightWheels.setVelocity(v, percent);
  LeftWheels.setVelocity(v, percent);
}

void turnDownPincers() {
  FrontPincers.spinFor(reverse, 340, degrees);
}

int myThreadCallback() {
  turnDownPincers();
    // Debes dormir los hilos utilizando el comando 'this_thread::sleep_for(unit in
    // mseg)' para evitar que este hilo utilice todos los recursos de la CPU
    // recursos de la CPU.
    this_thread::sleep_for(25);
  
  // El callback de un hilo debe devolver un int, aunque el código nunca
  // llegue aquí. Debe devolver un int aquí. Los hilos pueden salir, pero este no lo hace.
  return 0;
}

void autonomousYellowGoal() {
  thread myThread = thread(myThreadCallback);
  Drivetrain.drive(forward);
}

void autonomousBackYellowGoal() {
  while(1) {
    if(FrontButton.pressing() || RightWheels.rotation(degrees) > 800) {
      Drivetrain.stop();
      wait(250, msec);
      FrontPincers.spinFor(forward, 140, degrees);
      changeVelocity(50);
      Drivetrain.driveFor(reverse, 35, inches);

      break;
    }
  }
}

void autonomousPlatform() {
  Drivetrain.turnToHeading(270, degrees);
  changeVelocity(50);
  Drivetrain.driveFor(forward, 100, inches);
  Drivetrain.turnToHeading(180, degrees);
  Drivetrain.driveFor(forward, 10, inches);
  Drivetrain.turnToHeading(270, degrees);
  BackPincers.spinFor(forward, 305, degrees);
  Drivetrain.driveFor(reverse, 30, inches);

}

void autonomous(void) {
  changeVelocity(100);
  FrontPincers.setVelocity(100, percent);
  RightWheels.resetRotation();
  LeftWheels.resetRotation();
  autonomousYellowGoal();
  autonomousBackYellowGoal();
  autonomousPlatform();
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
  FrontPincers.setVelocity(80, percent);
  BackPincers.setVelocity(80, percent);
  FrontPincers.setMaxTorque(100, percent);
  BackPincers.setMaxTorque(100, percent);
  while (1) {
    changeMovement();
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
