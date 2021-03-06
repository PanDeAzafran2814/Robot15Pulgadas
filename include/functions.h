#include "devices.h"

void initConfig(int v, int t) {
  RightWheels.setMaxTorque(t, percent);
  LeftWheels.setMaxTorque(t,percent);
  FrontPincers.setVelocity(v, percent);
  BackPincers.setVelocity(v, percent);
  RightWheels.resetRotation();
  LeftWheels.resetRotation();
  Drivetrain.setRotation(0, degrees);
  FrontPincers.setRotation(0, degrees);
  BackPincers.setRotation(0, degrees);
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

void changeMovement() {
  if(Control.ButtonX.pressing()) {
    isReverse = true;
  }
  if(Control.ButtonB.pressing()) {
    isReverse = false;
  }
}

void valveManagement(){
  if(Control.ButtonA.pressing()) {
    BackValve.set(0);
  }
  if(Control.ButtonY.pressing()) {
    BackValve.set(1);
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
  if(Control.ButtonRight.pressing()) {
    bandaControl = 1;
  }
  else if(Control.ButtonLeft.pressing()) {
    bandaControl = 2;
  } else if(Control.ButtonUp.pressing()){
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

void useDistanceSensor(directionType direction,int Inches) {
  bool enableDistance = true;
  while(enableDistance){
    if(direction == forward) {
      if(DistanceSensor.objectDistance(inches) < Inches || !DistanceSensor.isObjectDetected()) {
        Drivetrain.drive(forward);
      } else {
        Drivetrain.stop();
        enableDistance = false;
      }
    }
    if(direction == reverse) {
      if(DistanceSensor.objectDistance(inches) > Inches || !DistanceSensor.isObjectDetected()) {
        Drivetrain.drive(reverse);
      } else {
        Drivetrain.stop();
        enableDistance = false;
      }
    }
    
  }
}