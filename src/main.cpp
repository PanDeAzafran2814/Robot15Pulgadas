#include "functions.h"


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


// Main Autonomo
void autonomous(void) {
  initConfig(100, 100);
  changeVelocity(100);
  thread handsThread = thread(moveArmsThread);
  
  bool enableDistance = true;
  while(enableDistance){
    if(DistanceSensor.objectDistance(inches) < 50 || !DistanceSensor.isObjectDetected()) {
      Drivetrain.drive(forward);
    } else {
      Drivetrain.stop();
      enableDistance = false;
    }
  }
  wait(150, msec);
  changeVelocity(70);
  FrontPincers.spinFor(forward, 180, degrees);
  enableDistance = true;
  while(enableDistance){
    if(DistanceSensor.objectDistance(inches) > 30 || !DistanceSensor.isObjectDetected()) {
      Drivetrain.drive(reverse);
    } else {
      Drivetrain.stop();
      enableDistance = false;
    }
  }
  changeVelocity(30);
  FrontValve.set(true);
  Drivetrain.turnFor(left, 90, degrees);
  BackPincers.spinFor(forward, 450, degrees);
  Drivetrain.driveFor(reverse, 11, inches);
  BackPincers.spinFor(reverse, 180, degrees);
}

void useDistanceSensor() {
  bool enableDistance = true;
  while(enableDistance){
    if(DistanceSensor.objectDistance(inches) > 30 || !DistanceSensor.isObjectDetected()) {
      Drivetrain.drive(reverse);
    } else {
      Drivetrain.stop();
      enableDistance = false;
    }
  }
}
/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  LeftWheels.setVelocity(100, percent);
  RightWheels.setVelocity(100, percent);
  FrontPincers.setVelocity(100, percent);
  BackPincers.setVelocity(100, percent);
  FrontPincers.setMaxTorque(100, percent);
  BackPincers.setMaxTorque(100, percent);
  while (1) {
    Control.Screen.print(DistanceSensor.objectDistance(inches));
    Control.Screen.newLine();
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
