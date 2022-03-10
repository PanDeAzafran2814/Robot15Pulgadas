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


// Main Skills
void autonomous(void) {
  initConfig(100, 100); // Init config
  changeVelocity(30); // Change velocity
  BackValve.set(true);
  BackPincers.spinFor(forward, 420, degrees); // Turn down front pincers
  useDistanceSensor(reverse, 3); // Drive reverse 10 inches with distance sensor
  wait(150, msec);
  BackPincers.spinFor(reverse, 230, degrees); // Pick up pincers to pick team goal
  BackValve.set(false);
  Drivetrain.driveFor(forward, 3, inches);
  Drivetrain.turnFor(right, 85, degrees);
  FrontPincers.spinFor(reverse, 380, degrees);
  useDistanceSensor(forward, 60); // Drive forward 50 inches
  FrontPincers.spinFor(forward, 180, degrees);
  Drivetrain.turnToHeading(270, degrees);
  useDistanceSensor(reverse, 30);
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
