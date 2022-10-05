/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\s725156                                          */
/*    Created:      Wed Sep 14 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// camera               vision        10              
// ---- END VEXCODE CONFIGURED DEVICES ----

// VEX V5 C++ Project
#include "vex.h"
#include <algorithm>
#include <cmath>
#include "camera.h"

using namespace vex;

//#region config_globals
vex::motor      back_right_motor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor      back_left_motor(vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::motor      front_right_motor(vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::motor      front_left_motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::controller con1(vex::controllerType::primary);
vex::controller con2(vex::controllerType::partner);
vex::inertial   Inertial2(vex::PORT9);
vex::motor      shooter_left(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor      shooter_right(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor      flicker(vex::PORT18, vex::gearSetting::ratio18_1, true);
vex::gps        gps(vex::PORT17, 0, turnType::right);
vex::vision     VisionSensor(vex::PORT10);

//#endregion config_globals

class Drives {
  public:
    void static robotOriented() {
      double maxSpeed = 100;
      //Get the raw sums of the X and Y joystick axes
      double front_left  = (double)(con1.Axis3.position(pct) - con1.Axis4.position(pct));
      double back_left   = (double)(con1.Axis3.position(pct) + con1.Axis4.position(pct));
      double front_right = (double)(con1.Axis3.position(pct) + con1.Axis4.position(pct));
      double back_right  = (double)(con1.Axis3.position(pct) - con1.Axis4.position(pct));
      
      //Find the largest possible sum of X and Y
      double max_raw_sum = (double)(abs(con1.Axis3.position(pct)) + abs(con1.Axis4.position(pct)));
      
      //Find the largest joystick value
      double max_XYstick_value = (double)(std::max(abs(con1.Axis3.position(pct)),abs(con1.Axis4.position(pct))));
      
      //The largest sum will be scaled down to the largest joystick value, and the others will be
      //scaled by the same amount to preserve directionality
      if (max_raw_sum != 0) {
          front_left  = front_left / max_raw_sum * max_XYstick_value;
          back_left   = back_left / max_raw_sum * max_XYstick_value;
          front_right = front_right / max_raw_sum * max_XYstick_value;
          back_right  = back_right / max_raw_sum * max_XYstick_value;
      }
      
      //Now to consider rotation
      //Naively add the rotational axis
      front_left  = front_left  - con1.Axis1.position(pct);
      back_left   = back_left   - con1.Axis1.position(pct);
      front_right = front_right + con1.Axis1.position(pct);
      back_right  = back_right  + con1.Axis1.position(pct);
      
      //What is the largest sum, or is 100 larger?
      max_raw_sum = std::max(std::abs(front_left),std::max(std::abs(back_left),std::max(std::abs(front_right),std::max(std::abs(back_right),maxSpeed))));
      
      //Scale everything down by the factor that makes the largest only 100, if it was over
      front_left  = front_left  / max_raw_sum * maxSpeed;
      back_left   = back_left   / max_raw_sum * maxSpeed;
      front_right = front_right / max_raw_sum * maxSpeed;
      back_right  = back_right  / max_raw_sum * maxSpeed;
      con1.Screen.clearScreen();
      con1.Screen.setCursor(1, 1);
      con1.Screen.print(front_left);
      //Write the manipulated values out to the motors
      front_left_motor.spin(fwd,front_left, velocityUnits::pct);
      back_left_motor.spin(fwd,back_left,  velocityUnits::pct);
      front_right_motor.spin(fwd,front_right,velocityUnits::pct);
      back_right_motor.spin(fwd,back_right, velocityUnits::pct);
    }

  public:
    void static fieldOriented() {
      double maxSpeed = 100;
      double headingRadians = Inertial2.heading() * 3.14159/180;
      double yPos = con2.Axis3.position(pct);
      double xPos = con2.Axis4.position(pct);
      double sineHeading = sin(headingRadians);
      double cosHeading = cos(headingRadians);
      double rotatedYPos = xPos * sineHeading + yPos * cosHeading;
      double rotatedXPos = xPos * cosHeading - yPos * sineHeading;
      
      //Get the raw sums of the X and Y joystick axes
      double front_left  = (double)(rotatedYPos + rotatedXPos);
      double back_left   = (double)(rotatedYPos - rotatedXPos);
      double front_right = (double)(rotatedYPos - rotatedXPos);
      double back_right  = (double)(rotatedYPos + rotatedXPos);

      
      //Find the largest possible sum of X and Y
      double max_raw_sum = (double)(abs(con2.Axis3.position(pct)) + abs(con2.Axis4.position(pct)));
      
      //Find the largest joystick value
      double max_XYstick_value = (double)(std::max(abs(con2.Axis3.position(pct)),abs(con2.Axis4.position(pct))));
      
      //The largest sum will be scaled down to the largest joystick value, and the others will be
      //scaled by the same amount to preserve directionality
      if (max_raw_sum != 0) {
        front_left  = front_left / max_raw_sum * max_XYstick_value;
        back_left   = back_left / max_raw_sum * max_XYstick_value;
        front_right = front_right / max_raw_sum * max_XYstick_value;
        back_right  = back_right / max_raw_sum * max_XYstick_value;
      }
      
      //Now to consider rotation
      //Naively add the rotational axis
      front_left  = front_left  - con2.Axis1.position(pct);
      back_left   = back_left   - con2.Axis1.position(pct);
      front_right = front_right + con2.Axis1.position(pct);
      back_right  = back_right  + con2.Axis1.position(pct);
      
      //What is the largest sum, or is 100 larger?
      max_raw_sum = std::max(std::abs(front_left),std::max(std::abs(back_left),std::max(std::abs(front_right),std::max(std::abs(back_right),maxSpeed))));
      
      //Scale everything down by the factor that makes the largest only 100, if it was over
      front_left  = front_left  / max_raw_sum * maxSpeed;
      back_left   = back_left   / max_raw_sum * maxSpeed;
      front_right = front_right / max_raw_sum * maxSpeed;
      back_right  = back_right  / max_raw_sum * maxSpeed;
      
      //Write the manipulated values out to the motors
      front_left_motor.spin(fwd,front_left, velocityUnits::pct);
      back_left_motor.spin(fwd,back_left,  velocityUnits::pct);
      front_right_motor.spin(fwd,front_right,velocityUnits::pct);
      back_right_motor.spin(fwd,back_right, velocityUnits::pct);
    }
};

int main(void) {
    Inertial2.setHeading(0.0, degrees);
    Inertial2.setRotation(0.0, degrees);
    Inertial2.startCalibration();
    while (Inertial2.isCalibrating()) { 
        task::sleep(10); 
    }
    Inertial2.setHeading(0.0, degrees);
    Inertial2.setRotation(0.0, degrees);
    int center = 158;
    while(true) {
      
      
      
      //Drives::robotOriented();
      
      VisionSensor.takeSnapshot(GOAL_RED);
      if ( con2.ButtonA.pressing() && VisionSensor.largestObject.exists ) {
        int mid = VisionSensor.largestObject.originX + (VisionSensor.largestObject.width / 2);
        int turning = (center - mid) * -0.2;
        front_left_motor.setVelocity(-turning, velocityUnits::pct);
        front_right_motor.setVelocity(turning, velocityUnits::pct);
        back_left_motor.setVelocity(-turning, velocityUnits::pct);
        back_right_motor.setVelocity(turning, velocityUnits::pct);
        front_left_motor.spin(directionType::fwd);
        front_right_motor.spin(directionType::fwd);
        back_left_motor.spin(directionType::fwd);
        back_right_motor.spin(directionType::fwd);
      } else {
        Drives::fieldOriented();
      }
      
      
      
  }


  
  
 
}

