/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\s725156                                          */
/*    Created:      Wed Sep 21 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <algorithm>
#include <cmath>
using namespace vex;

//#region config_globals
vex::motor      back_right_motor(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor      back_left_motor(vex::PORT4, vex::gearSetting::ratio18_1, true);
vex::motor      front_right_motor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor      front_left_motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::controller con1(vex::controllerType::primary);
vex::gps        GPS(vex::PORT11, 0, turnType::right);
vex::inertial   Inertial2(vex::PORT9);
class Auton {
  public:
    void static fieldOrientedAuton(int frontback, int sideways, int turning) {
      double maxSpeed = 100;
      double headingRadians = Inertial2.heading() * 3.14159/180;
      double yPos = frontback;
      double xPos = sideways;
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
      double max_raw_sum = (double)(abs(frontback) + abs(sideways));
      
      //Find the largest joystick value
      double max_XYstick_value = (double)(std::max(abs(frontback),abs(sideways)));
      
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
      front_left  = front_left  + turning;
      back_left   = back_left   + turning;
      front_right = front_right - turning;
      back_right  = back_right  - turning;
      
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

  public:
    // in inches and degrees
    void static goTo(int xToGo, int yToGo, int endOrientation, int secondsToComplete) {
      double WHEEL_DIAMETER = 4; // inches
      double CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;

      // 84 teeth on wheels and 36 on motors
      // also 7/3 gear ratio
      double GEAR_RATIO = 84/36;
      double x_rotations = (xToGo / CIRCUMFERENCE) * GEAR_RATIO;
      double y_rotations = (yToGo / CIRCUMFERENCE) * GEAR_RATIO;
      /*
      *     robot wheels looks like this:
      *   /   \
      *   \   /
      *
      *   they vector outward at 45 degrees each
      *  however, we want to drive forward a certain amount of inches
      * not driving 45 degrees in a direction
      * if you extend everything out then you get a square
      * with a line going from one corner to another that is as far as you want
      * your robot to drive.
      * You get a 45 45 90 triangle, where the legs are each x and the hypotenuse is x root 2
      * that means each of the legs is the distance you want to go divided by root 2
      * so we have each of the robot motors turn that far instead
      * 
      * then we multiply by 1.1 to overshoot a little bit because our motors have play and are bad
      */
      double xDegrees = x_rotations * 360 / sqrt(2) * 1.1;
      double yDegrees = y_rotations * 360  / sqrt(2) * 1.1;
      
      front_left_motor.resetRotation();
      back_left_motor.resetRotation();
      front_right_motor.resetRotation();
      back_right_motor.resetRotation();


      // different direction where the wheels are pointing so different degrees to turn
      double front_left_degrees = xDegrees + yDegrees;
      double front_right_degrees = xDegrees - yDegrees;
      double back_left_degrees = xDegrees - yDegrees;
      double back_right_degrees = xDegrees + yDegrees;

      double front_left_degrees_per_second = front_left_degrees/secondsToComplete;
      double front_right_degrees_per_second = front_right_degrees/secondsToComplete;
      double back_left_degrees_per_second = back_left_degrees/secondsToComplete;
      double back_right_degrees_per_second = back_right_degrees/secondsToComplete;

      while ( front_right_motor.rotation(deg) < front_right_degrees || front_left_motor.rotation(deg) < front_left_degrees || back_left_motor.rotation(deg) < back_left_degrees || back_right_motor.rotation(deg) < back_right_degrees ) {
        // set turning thingy here
        front_left_motor.spin(fwd, front_left_degrees_per_second, dps);
        back_left_motor.spin(fwd, back_left_degrees_per_second, dps);
        front_right_motor.spin(fwd, front_right_degrees_per_second, dps);
        back_right_motor.spin(fwd, back_right_degrees_per_second, dps);
        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print(xDegrees);
        con1.Screen.setCursor(1, 10);
        con1.Screen.print(front_right_motor.rotation(deg));
      }
      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();
    }
};


// field is -1800 to 1800 in mm and is like that in coordinate system

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit(); 

  // reset gyro
  Inertial2.setHeading(0.0, degrees);
  Inertial2.setRotation(0.0, degrees);
  Inertial2.startCalibration();
  while (Inertial2.isCalibrating()) { 
      task::sleep(10); 
  }
  Inertial2.setHeading(0.0, degrees);
  Inertial2.setRotation(0.0, degrees);
  
  

  //Auton::fieldOrientedAuton(0, 30, 10);
  Auton::goTo(24, 24, 0, 5);

  

  
}
