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
// DigitalOutA          digital_out   A               
// DigitalOutH          digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----

// VEX V5 C++ Project
#include "vex.h"
#include <algorithm>
#include <cmath>
#include "camera.h"
#include "PID.cpp"

using namespace vex;

//#region config_globals
vex::motor      back_right_motor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor      back_left_motor(vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::motor      front_right_motor(vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::motor      front_left_motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor      intakeLeft(vex::PORT6, vex::gearSetting::ratio18_1, false);
vex::motor      intakeRight(vex::PORT5, vex::gearSetting::ratio18_1, false);
vex::controller con1(vex::controllerType::primary);
vex::controller con2(vex::controllerType::partner);
vex::inertial   Inertial2(vex::PORT9);
vex::motor      shooter_left(vex::PORT7, vex::gearSetting::ratio18_1, true);
vex::motor      shooter_right(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::gps        gps(vex::PORT17, 0, turnType::right);
vex::vision     VisionSensor(vex::PORT10);
vex::motor      expansion(vex::PORT20);
vex::competition Competition;
vex::timer vexTimer;
vex::optical colorSensor(vex::PORT11, false);



//#endregion config_globals

class Drives {
  public:
    void static robotOriented() {
      double maxSpeed = 100;
      double yPos = con2.Axis3.position(pct);
      if ( yPos < 0 ) {
        yPos = (yPos/10)*(yPos/10) * -1;
      } else {
        yPos = (yPos/10)*(yPos/10);
      }
      double xPos = con2.Axis4.position(pct);
      if ( xPos < 0 ) {
        xPos = (xPos/10)*(xPos/10) * -1;
      } else {
        xPos = (xPos/10)*(xPos/10);
      }
      //Get the raw sums of the X and Y joystick axes
      double front_left  = (double)(yPos + xPos);
      double back_left   = (double)(yPos - xPos);
      double front_right = (double)(yPos - xPos);
      double back_right  = (double)(yPos + xPos);
      
      //Find the largest possible sum of X and Y
      double max_raw_sum = (double)(abs((int)(yPos)) + abs((int)(xPos)));
      
      //Find the largest joystick value
      double max_XYstick_value = (double)(std::max(abs((int)(yPos)),abs((int)(xPos))));
      
      //The largest sum will be scaled down to the largest joystick value, and the others will be
      //scaled by the same amount to preserve directionality
      if (max_raw_sum != 0) {
          front_left  = front_left / max_raw_sum * max_XYstick_value;
          back_left   = back_left / max_raw_sum * max_XYstick_value;
          front_right = front_right / max_raw_sum * max_XYstick_value;
          back_right  = back_right / max_raw_sum * max_XYstick_value;
      }


      double turning = con2.Axis1.position(pct);
      if ( turning < 0 ) {
        turning = (turning/10)*(turning/10) * -1;
      } else {
        turning = (turning/10)*(turning/10);
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
    void static fieldOriented() {
      double maxSpeed = 12;
      double headingRadians = Inertial2.heading() * 3.14159/180;
      double yPos = con1.Axis3.position(pct);
      if ( yPos < 0 ) {
        yPos = (yPos/10)*(yPos/10) * -1;
      } else {
        yPos = (yPos/10)*(yPos/10);
      }
      double xPos = con1.Axis4.position(pct);
      if ( xPos < 0 ) {
        xPos = (xPos/10)*(xPos/10) * -1;
      } else {
        xPos = (xPos/10)*(xPos/10);
      }
      double sineHeading = sin(headingRadians);
      double cosHeading = cos(headingRadians);
      double rotatedYPos = xPos * sineHeading + yPos * cosHeading;
      double rotatedXPos = xPos * cosHeading - yPos * sineHeading;

      if ( con1.ButtonDown.pressing() ) {
        rotatedYPos = -7.5;
      }
      
      //Get the raw sums of the X and Y joystick axes
      double front_left  = (double)(rotatedYPos + rotatedXPos);
      double back_left   = (double)(rotatedYPos - rotatedXPos);
      double front_right = (double)(rotatedYPos - rotatedXPos);
      double back_right  = (double)(rotatedYPos + rotatedXPos);

      
      //Find the largest possible sum of X and Y
      double max_raw_sum = (double)(abs((int)(rotatedYPos)) + abs((int)(rotatedXPos)));
      
      //Find the largest joystick value
      double max_XYstick_value = (double)(std::max(abs((int)(rotatedYPos)),abs((int)(rotatedXPos))));
      
      //The largest sum will be scaled down to the largest joystick value, and the others will be
      //scaled by the same amount to preserve directionality
      if (max_raw_sum != 0) {
        front_left  = front_left / max_raw_sum * max_XYstick_value;
        back_left   = back_left / max_raw_sum * max_XYstick_value;
        front_right = front_right / max_raw_sum * max_XYstick_value;
        back_right  = back_right / max_raw_sum * max_XYstick_value;
      }
      
      double turning = con1.Axis1.position(pct);
      double magicNumber;
      if ( abs((int)turning) < 93 ) {
        magicNumber = 1.025;
      } else {
        magicNumber = 1.048;
      }
      if ( turning < 0 ) {
        turning = pow(magicNumber, -turning)-1;
        turning *= -1;
      } else {
        turning = pow(magicNumber, turning)-1;
      }
      

      //Now to consider rotation
      //Naively add the rotational axis
      front_left  = front_left  - turning;
      back_left   = back_left   - turning;
      front_right = front_right + turning;
      back_right  = back_right  + turning;
      
      //What is the largest sum, or is 100 larger?
      max_raw_sum = std::max(std::abs(front_left),std::max(std::abs(back_left),std::max(std::abs(front_right),std::max(std::abs(back_right),maxSpeed))));
      
      //Scale everything down by the factor that makes the largest only 100, if it was over
      front_left  = front_left  / max_raw_sum * maxSpeed;
      back_left   = back_left   / max_raw_sum * maxSpeed;
      front_right = front_right / max_raw_sum * maxSpeed;
      back_right  = back_right  / max_raw_sum * maxSpeed;
      
      //Write the manipulated values out to the motors
      front_left_motor.spin(fwd,front_left, voltageUnits::volt);
      back_left_motor.spin(fwd,back_left,  voltageUnits::volt);
      front_right_motor.spin(fwd,front_right, voltageUnits::volt);
      back_right_motor.spin(fwd,back_right, voltageUnits::volt);
    }
};

class AutonCommands {

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
      double GEAR_RATIO = 1/1;
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

  public:
    // in inches and degrees
    void static goTo(double xToGo, double yToGo, double secondsToComplete) {
      xToGo /= 2;
      yToGo /= 2;

      // forward and right are positive
      int leftBackSign = 1;
      int rightFrontSign = 1;

      // forward and left are positive
      int rightBackSign = 1;
      int leftFrontSign = 1;



      if ( yToGo < -xToGo ) {
        leftFrontSign = -1;
        rightBackSign = -1;
      }

      if ( yToGo > xToGo ) {
        leftBackSign = -1;
        rightFrontSign = -1;
      }
      

      double WHEEL_DIAMETER = 4; // inches
      double CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER;
      vexTimer.clear();

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
      double xDegrees = x_rotations * 360 / sqrt(2);
      double yDegrees = y_rotations * 360  / sqrt(2);
      
      front_left_motor.resetRotation();
      back_left_motor.resetRotation();
      front_right_motor.resetRotation();
      back_right_motor.resetRotation();

      // different direction where the wheels are pointing so different degrees to turn
      double front_left_degrees = xDegrees + yDegrees;
      double front_right_degrees = xDegrees - yDegrees;
      double back_left_degrees = xDegrees - yDegrees;
      double back_right_degrees = xDegrees + yDegrees;
      con1.Screen.print(front_left_degrees);

      double front_left_degrees_per_second = front_left_degrees/secondsToComplete;
      double front_right_degrees_per_second = front_right_degrees/secondsToComplete;
      double back_left_degrees_per_second = back_left_degrees/secondsToComplete;
      double back_right_degrees_per_second = back_right_degrees/secondsToComplete;
      double error = 10;

      front_left_motor.spin(fwd, front_left_degrees_per_second, dps);
      back_left_motor.spin(fwd, back_left_degrees_per_second, dps);
      front_right_motor.spin(fwd, front_right_degrees_per_second, dps);
      back_right_motor.spin(fwd, back_right_degrees_per_second, dps);

      if ( front_left_degrees >= 0 ) {
        if ( front_right_degrees >= 0 ) {
          while((vexTimer.time(sec) < secondsToComplete + 2) && (front_right_motor.rotation(deg) < front_right_degrees || back_left_motor.rotation(deg) < back_left_degrees || front_left_motor.rotation(deg) < front_left_degrees || back_right_motor.rotation(deg) < back_right_degrees)){
            con1.Screen.clearScreen();
            con1.Screen.setCursor(1, 1);
            con1.Screen.print(xDegrees);
            con1.Screen.setCursor(1, 10);
            con1.Screen.print(front_right_motor.rotation(deg));
          }
        } else {
          while((vexTimer.time(sec) < secondsToComplete + 2) && (front_right_motor.rotation(deg) > front_right_degrees || back_left_motor.rotation(deg) > back_left_degrees || front_left_motor.rotation(deg) < front_left_degrees || back_right_motor.rotation(deg) < back_right_degrees)){
            con1.Screen.clearScreen();
            con1.Screen.setCursor(1, 1);
            con1.Screen.print(xDegrees);
            con1.Screen.setCursor(1, 10);
            con1.Screen.print(front_right_motor.rotation(deg));
          }
        }
      } else {
        if ( front_right_degrees >= 0 ) {
          while((vexTimer.time(sec) < secondsToComplete + 2) && (front_right_motor.rotation(deg) < front_right_degrees || back_left_motor.rotation(deg) < back_left_degrees || front_left_motor.rotation(deg) > front_left_degrees || back_right_motor.rotation(deg) > back_right_degrees)){
            con1.Screen.clearScreen();
            con1.Screen.setCursor(1, 1);
            con1.Screen.print(xDegrees);
            con1.Screen.setCursor(1, 10);
            con1.Screen.print(front_right_motor.rotation(deg));
          }
        } else {
          while((vexTimer.time(sec) < secondsToComplete + 2) && (front_right_motor.rotation(deg) > front_right_degrees || back_left_motor.rotation(deg) > back_left_degrees || front_left_motor.rotation(deg) > front_left_degrees || back_right_motor.rotation(deg) > back_right_degrees)){
            con1.Screen.clearScreen();
            con1.Screen.setCursor(1, 1);
            con1.Screen.print(xDegrees);
            con1.Screen.setCursor(1, 10);
            con1.Screen.print(front_right_motor.rotation(deg));
          }
        }
      }

      
      //    while((vexTimer.time(sec) < secondsToComplete + 2)&&(!rangeChecker(front_right_motor.rotation(deg), front_right_degrees, error) || !rangeChecker(front_left_motor.rotation(deg), front_left_degrees, error) || !rangeChecker(back_left_motor.rotation(deg), back_left_degrees, error) || !rangeChecker(back_right_motor.rotation(deg), back_right_degrees, error))){
      //      con1.Screen.clearScreen();
      //      con1.Screen.setCursor(1, 1);
      //      con1.Screen.print(xDegrees);
      //      con1.Screen.setCursor(1, 10);
      //      con1.Screen.print(front_right_motor.rotation(deg));
      //    }

      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();
    }
  
  public:
    bool static rangeChecker(double curr, double target, double error){
      return abs((int) curr - (int) target) < error;
    }

  public:
    // speed in percentage
    void static turnTo(int degreesToTurn, int speed) {
      front_left_motor.setBrake(brakeType::brake);
      front_right_motor.setBrake(brakeType::brake);
      back_left_motor.setBrake(brakeType::brake);
      back_right_motor.setBrake(brakeType::brake);
      
      front_left_motor.setStopping(brakeType::brake);
      front_right_motor.setStopping(brakeType::brake);
      back_left_motor.setStopping(brakeType::brake);
      back_right_motor.setStopping(brakeType::brake);

      front_left_motor.setVelocity(speed, velocityUnits::pct);
      back_left_motor.setVelocity(speed, velocityUnits::pct);
      front_right_motor.setVelocity(-speed, velocityUnits::pct);
      back_right_motor.setVelocity(-speed, velocityUnits::pct);

      front_left_motor.spin(directionType::fwd);
      back_left_motor.spin(directionType::fwd);
      front_right_motor.spin(directionType::fwd);
      back_right_motor.spin(directionType::fwd);
      int start_angle = -Inertial2.orientation(yaw, deg);
      int end_angle = start_angle + degreesToTurn;
      if ( degreesToTurn < 0 ) {
        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print("turn left");
        while ( -Inertial2.orientation(yaw, deg) > end_angle ) {
          con1.Screen.clearScreen();
          con1.Screen.setCursor(1, 1);
          con1.Screen.print("turning left");
        }
      } else  {
        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print("turn right");
        while ( -Inertial2.orientation(yaw, deg) < end_angle ) {
          con1.Screen.clearScreen();
          con1.Screen.setCursor(1, 1);
          con1.Screen.print(Inertial2.orientation(yaw, deg));
          con1.Screen.setCursor(1, 2);
          con1.Screen.print(end_angle);
        }
      }
      con1.Screen.clearScreen();
      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();
    }

  public:
    void static runIntake(double secondsToComplete, bool) {
      intakeLeft.setBrake(coast);
      intakeRight.setBrake(coast);
      intakeLeft.setReversed(true);
      intakeLeft.setVelocity(100, velocityUnits::pct);
      intakeRight.setVelocity(100, velocityUnits::pct);
      intakeLeft.spinFor(secondsToComplete, sec);
      intakeRight.spinFor(secondsToComplete, sec);
    }

  public:
    void static doRoller(){
      double secondsToComplete = 0.5;
      intakeLeft.setBrake(coast);
      intakeRight.setBrake(coast);
      intakeLeft.setReversed(false);
      //intakeRight.setReversed(true);
      intakeLeft.setVelocity(100, velocityUnits::pct);
      //intakeRight.setVelocity(100, velocityUnits::pct);

      intakeLeft.spin(directionType::fwd);
      //intakeRight.spin(directionType::fwd);


      vexTimer.clear();
      while(vexTimer.time(sec) < secondsToComplete) {

      }

      intakeLeft.stop();
      //intakeRight.stop();

    }
  
  /*
  positive velocity = shooter side front
  .25 = 2 inches
  .5 = 7 inches
  .75 = 17 inches
  1 = 26 inches
  1.5 = 45 inches (two tiles)
  */
  public:
    void static upALittle( int speed, double secondsToComplete){


      front_left_motor.setVelocity(speed, velocityUnits::pct);
      back_left_motor.setVelocity(speed , velocityUnits::pct);
      front_right_motor.setVelocity(speed , velocityUnits::pct);
      back_right_motor.setVelocity(speed , velocityUnits::pct);

      front_left_motor.spin(directionType::fwd);
      back_left_motor.spin(directionType::fwd);
      front_right_motor.spin(directionType::fwd);
      back_right_motor.spin(directionType::fwd);

      vexTimer.clear();
      while(vexTimer.time(sec) < secondsToComplete) {

      }

      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();
    }

  /*
  false = clockwise
  .25 = 45 degrees
  .365 = 90 degrees
  .6 = 135 degrees
  .725 = 180 degrees
  */
  public:
    void static turning(bool rev,double secondsToComplete){
        
        int speed = 100;
        if(rev){
          speed *= -1;
        }
        front_left_motor.setVelocity(-speed, velocityUnits::pct);
        back_left_motor.setVelocity(-speed, velocityUnits::pct);
        front_right_motor.setVelocity(speed, velocityUnits::pct);
        back_right_motor.setVelocity(speed, velocityUnits::pct);

        front_left_motor.spin(directionType::fwd);
        back_left_motor.spin(directionType::fwd);
        front_right_motor.spin(directionType::fwd);
        back_right_motor.spin(directionType::fwd);

        vexTimer.clear();
        while(vexTimer.time(sec) < secondsToComplete) {

        }

        front_left_motor.stop();
        back_left_motor.stop();
        front_right_motor.stop();
        back_right_motor.stop();
    }

  //positive velocity = right with intake as front
  public:
    void static leftyRighty(int speed, double secondsToComplete){
      front_left_motor.setVelocity(-speed, velocityUnits::pct);
      back_left_motor.setVelocity(speed, velocityUnits::pct);
      front_right_motor.setVelocity(speed, velocityUnits::pct);
      back_right_motor.setVelocity(-speed, velocityUnits::pct);

      front_left_motor.spin(directionType::fwd);
      back_left_motor.spin(directionType::fwd);
      front_right_motor.spin(directionType::fwd);
      back_right_motor.spin(directionType::fwd);

      vexTimer.clear();
      while(vexTimer.time(sec) < secondsToComplete) {

      }

      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();
    }
  
  public:
    void static spinIntake(){
      intakeLeft.setBrake(coast);
      intakeRight.setBrake(coast);
      intakeLeft.setReversed(true);
      intakeRight.setReversed(false);
      intakeLeft.setVelocity(100, velocityUnits::pct);
      intakeRight.setVelocity(100, velocityUnits::pct);

      intakeLeft.spin(directionType::fwd);
      intakeRight.spin(directionType::fwd);

    }

  public:
    void static stopIntake(){

      intakeLeft.stop();
      intakeRight.stop();

    }

  public:
    void static aimAndShoot(){
      AutonCommands::aim();

      AutonCommands::shoot(2);
      AutonCommands::shoot(1);
      AutonCommands::shoot(1);

      AutonCommands::stopShooters();

    }

  public:
    void static aim(){
      PID goal(0.6, 1, 1, 0);
      int screenCenter = 158;
      vex::timer timerThing;
      timerThing.clear();
      timerThing.reset();

      while(timerThing.time(sec) < 0.8 ) {
        VisionSensor.takeSnapshot(GOAL_RED);
        if ( VisionSensor.largestObject.exists) {
          int targetMid = VisionSensor.largestObject.originX + (VisionSensor.largestObject.width / 2);
          // other random crap value is 'feedforward' stfu its messed up
          int error = screenCenter - targetMid;
          goal.setValues(0.2, 0.003, 0, targetMid);
          int turning = error * -0.2;

          turning += -2;


          front_left_motor.setVelocity(-turning, velocityUnits::pct);
          front_right_motor.setVelocity(turning, velocityUnits::pct);
          back_left_motor.setVelocity(-turning, velocityUnits::pct);
          back_right_motor.setVelocity(turning, velocityUnits::pct);
          front_left_motor.spin(directionType::fwd);
          front_right_motor.spin(directionType::fwd);
          back_left_motor.spin(directionType::fwd);
          back_right_motor.spin(directionType::fwd);
        } else {
          front_left_motor.stop();
          front_right_motor.stop();
          back_left_motor.stop();
          back_right_motor.stop();
        }
      }
      front_left_motor.stop();
      front_right_motor.stop();
      back_left_motor.stop();
      back_right_motor.stop();
      

    }

  public:
    static void shoot(double waitTime){
        // PID flywheelSpeed(0.4, 0, 0.3, 0);
        // int speed = 0;
        // int lastSpeed = 0;
        // lastSpeed = speed;
        // speed = 0.0028*(VisionSensor.largestObject.width-153.886)*(VisionSensor.largestObject.width-153.886)+28.544;
        // double percentSpeed = (shooter_left.velocity(velocityUnits::pct) + shooter_right.velocity(velocityUnits::pct))/2;
        VisionSensor.takeSnapshot(GOAL_RED); 
        int runAt = 0;
        if ( VisionSensor.largestObject.width > 80 ) {
          runAt = 9;
        } else if ( VisionSensor.largestObject.width > 50 ) {
          runAt = 9;
        } else if ( VisionSensor.largestObject.width > 35) {
          runAt = 10;
        } else {
          runAt = 11;
        }
        
        shooter_left.setBrake(coast);
        shooter_right.setBrake(coast);

        shooter_left.spin(directionType::fwd, runAt, voltageUnits::volt);
        shooter_right.spin(directionType::fwd, runAt, voltageUnits::volt);

        vexTimer.clear();
        while(vexTimer.time(sec) < waitTime) {

        }


        DigitalOutA.set(true);
        vexTimer.clear();
        while(vexTimer.time(sec) < 0.45
        ) {

        }
        DigitalOutA.set(false);
    }

  public:
    static void stopShooters(){
        shooter_left.setVelocity(0, velocityUnits::pct);
        shooter_right.setVelocity(0, velocityUnits::pct);
        shooter_left.spin(directionType::fwd);
        shooter_right.spin(directionType::fwd);
    }
    

  public:
    static void expand(){
      DigitalOutH.set(true);

      vexTimer.clear();
        while(vexTimer.time(sec) < 5) {
          
        }
      DigitalOutH.set(false);
    }

  public:
    static void wait(double secs){
     vexTimer.clear();
        while(vexTimer.time(sec) < secs) {
          
        }
    }

  public:
    static void starting(){
      //leftyRighty(100, 0.125);
      upALittle(-100, .25);
    }

  public:
    static void diagonal(double secondsToComplete){
      back_left_motor.setVelocity(100, velocityUnits::pct);
      front_right_motor.setVelocity(100, velocityUnits::pct);
      back_left_motor.spin(directionType::fwd);
      front_right_motor.spin(directionType::fwd);
      vexTimer.clear();
      while(vexTimer.time(sec) < secondsToComplete) {

      }
      back_left_motor.stop();
      front_right_motor.stop();


    }

  public:
    void static doRollerfast(){
      double secondsToComplete = 0.3;
      intakeLeft.setBrake(coast);
      intakeRight.setBrake(coast);
      intakeLeft.setReversed(false);
      //intakeRight.setReversed(true);
      intakeLeft.setVelocity(100, velocityUnits::pct);
      //intakeRight.setVelocity(100, velocityUnits::pct);

      intakeLeft.spin(directionType::fwd);
      //intakeRight.spin(directionType::fwd);


      vexTimer.clear();
      while(vexTimer.time(sec) < secondsToComplete) {

      }

      intakeLeft.stop();
      //intakeRight.stop();

    }

  public:
    void static spinUpFlywheel() {
      shooter_left.spin(directionType::fwd, 9, voltageUnits::volt);
        shooter_right.spin(directionType::fwd, 9, voltageUnits::volt);
      shooter_left.spin(directionType::fwd);
      shooter_right.spin(directionType::fwd);
    }

  public:
    void static intakeThree(double dist){
      AutonCommands::upALittle(-100, dist);
      AutonCommands::upALittle(75, 0.3);
      AutonCommands::spinIntake();
      AutonCommands::upALittle(-7, 6);
      AutonCommands::wait(0.5);
      AutonCommands::stopIntake();
    }

    
};

void auton1(void) {
  
  AutonCommands::goTo(0, -3, 0.4);
  AutonCommands::spinUpFlywheel();
  AutonCommands::starting();
  AutonCommands::doRollerfast();
  
  AutonCommands::goTo(2.5, 4, 0.3);
  AutonCommands::turnTo(5, 5);
  AutonCommands::shoot(2);
  AutonCommands::shoot(1);
  AutonCommands::stopShooters();
  AutonCommands::turnTo(100, 40);
  AutonCommands::spinIntake();
  AutonCommands::goTo(-50, 0, 3);
  AutonCommands::spinUpFlywheel();
  AutonCommands::turnTo(-73, -60);
  AutonCommands::shoot(2.3);
  AutonCommands::shoot(1.4);
  AutonCommands::stopShooters();
  AutonCommands::stopIntake();

  
}


void auton(void){
  //first three
  AutonCommands::starting();
  AutonCommands::doRoller();
  AutonCommands::upALittle( 100, .2);
  AutonCommands::leftyRighty(-50, .2);
  AutonCommands::turning(false, 0.6);
  AutonCommands::spinIntake();
  AutonCommands::upALittle( -50, 1);
  AutonCommands::turning(true, .15);
  AutonCommands::stopIntake();
  AutonCommands::upALittle( -100, .5);
  AutonCommands::doRoller();
  AutonCommands::upALittle(100, .25);
  AutonCommands::spinIntake();
  AutonCommands::diagonal(.8);
  AutonCommands::turning(true, .365);
  AutonCommands::spinUpFlywheel();
  AutonCommands::upALittle(75, 0.65);
  AutonCommands::stopIntake();
  AutonCommands::aimAndShoot();

  //second three

  AutonCommands::upALittle(-50, 0.2);
  AutonCommands::turning(true, .615);
  AutonCommands::spinIntake();
  AutonCommands::upALittle(-20, 3.85);
  AutonCommands::turning(false, .345);
  AutonCommands::wait(2);
  AutonCommands::stopIntake();
  AutonCommands::aimAndShoot();


  //third three
  AutonCommands::turning(true, .4);
  AutonCommands::wait(0.25);
  AutonCommands::intakeThree(1.4);
  AutonCommands::turning(false, 0.20);
  AutonCommands::upALittle(100, 0.5);
  AutonCommands::aimAndShoot();

  //fourth three
  AutonCommands::upALittle(-50, 0.4);
  AutonCommands::turning(false, .25);
  AutonCommands::intakeThree(0.85);

}

void driverAuton() {
  AutonCommands::starting();
  AutonCommands::doRoller();
  //AutonCommands::wait(1);
  AutonCommands::upALittle( 100, .2);
  //AutonCommands::wait(1);
  AutonCommands::leftyRighty(-50, .2);
  //AutonCommands::wait(1);
  //AutonCommands::turning(false, 0.63);
  AutonCommands::turnTo(60, 50);
  //AutonCommands::wait(1);
  AutonCommands::spinIntake();
  AutonCommands::upALittle( -50, 1);
  AutonCommands::turning(true, .15);
  AutonCommands::stopIntake();
  AutonCommands::upALittle( -100, .5);
  AutonCommands::doRoller();
  AutonCommands::upALittle(100, .25);
  AutonCommands::spinIntake();
  AutonCommands::diagonal(.65);
  AutonCommands::turning(true, .365);
  AutonCommands::wait(0.5);
  AutonCommands::stopIntake();
  //AutonCommands::turning(true, 0.1);
  AutonCommands::aimAndShoot();
  AutonCommands::turning(true, .6);
  AutonCommands::spinIntake();
  AutonCommands::upALittle(-25, 3.75);
  AutonCommands::turning(false, .345);
  AutonCommands::wait(2);
  AutonCommands::stopIntake();
  AutonCommands::aimAndShoot();
  AutonCommands::turning(true, .31);
  AutonCommands::upALittle(-100, 1);
  AutonCommands::upALittle(100, 0.7);
  AutonCommands::spinIntake();
  AutonCommands::upALittle(-25, 3.5);

  AutonCommands::stopIntake();
  AutonCommands::aimAndShoot();

}

void driving(void) {  

    //driverAuton();

    shooter_left.setBrake(coast);
    shooter_right.setBrake(coast);
    intakeLeft.setBrake(coast);
    intakeRight.setBrake(coast);
    intakeLeft.setReversed(true);
    intakeLeft.setVelocity(100, velocityUnits::pct);
    intakeRight.setVelocity(100, velocityUnits::pct);

    int screenCenter = 158;
    PID goal(0.6, 1, 1, 0);
    PID flywheelSpeed(0.4, 0, 0.3, 0);
    int speed = 0;
    int lastSpeed = 0;
    int goalcol = 0;
    while(true) {

      // resetting gyro if anything bad happens
      // intake is considered front, so have to reset while intake is facing away from driver/facing the front
      if ( con1.ButtonY.pressing() ) {
        Inertial2.setHeading(0.0, degrees);
        Inertial2.setRotation(0.0, degrees);
        Inertial2.startCalibration();
        while (Inertial2.isCalibrating()) { 
            task::sleep(10); 
        }
        Inertial2.setHeading(0.0, degrees);
        Inertial2.setRotation(0.0, degrees);
      } else if ( con1.ButtonRight.pressing() || con1.ButtonLeft.pressing() || con1.ButtonUp.pressing() ) {
        // turn to specific angle on second controller for rollers
        int robotHeading = Inertial2.heading();
        int turning = 0;
        if ( con1.ButtonLeft.pressing() ) {
          if ( (robotHeading - 90) % 360 > (90 - robotHeading) % 360 ) {
            turning = (90 - robotHeading) % 360;
          } else {
            turning = -((robotHeading - 90) % 360);
          }
        } else if ( con1.ButtonUp.pressing() ) {
          if ( (robotHeading - 180) % 360 > (180 - robotHeading) % 360 ) {
            turning = (180 - robotHeading) % 360;
          } else {
            turning = -((robotHeading - 180) % 360);
          }
        } else if ( con1.ButtonRight.pressing() ) {
          if ( (robotHeading - 270) % 360 > (270 - robotHeading) % 360 ) {
            turning = (270 - robotHeading) % 360;
          } else {
            turning = -((robotHeading - 270) % 360);
          }
        }
        front_left_motor.setVelocity(-turning, velocityUnits::pct);
        front_right_motor.setVelocity(turning, velocityUnits::pct);
        back_left_motor.setVelocity(-turning, velocityUnits::pct);
        back_right_motor.setVelocity(turning, velocityUnits::pct);
        front_left_motor.spin(directionType::fwd);
        front_right_motor.spin(directionType::fwd);
        back_left_motor.spin(directionType::fwd);
        back_right_motor.spin(directionType::fwd);
      } else {

        goalcol = 0;
        
        if  (con1.ButtonL2.pressing()|| con2.ButtonL2.pressing()) {
          VisionSensor.takeSnapshot(GOAL_RED);
          if ( VisionSensor.largestObject.exists ) {
            goalcol = 1;
          } else {
            VisionSensor.takeSnapshot(GOAL_BLUE);
            if ( VisionSensor.largestObject.exists ) {
              goalcol = 2;
            }
          }
        }
        if ( goalcol != 0 ) {
          /// aim at goal
          if ( (con1.ButtonL2.pressing()|| con2.ButtonL2.pressing()) && VisionSensor.largestObject.exists ) {
            
            // middle of targetted object
            int targetMid = VisionSensor.largestObject.originX + (VisionSensor.largestObject.width / 2);
            // other random crap value is 'feedforward' stfu its messed up
            int error = screenCenter - targetMid;
            goal.setValues(0.2, 0.003, 0, targetMid);
            int turning;
            

            if ( error < 20 ) {
              turning = goal.getOutput(screenCenter, 0, true, targetMid);
            } else {
              goal.resetError();
              turning = goal.getOutput(screenCenter, 0, false, targetMid);
            }
            
            turning += -2;

            front_left_motor.setVelocity(-turning, velocityUnits::pct);
            front_right_motor.setVelocity(turning, velocityUnits::pct);
            back_left_motor.setVelocity(-turning, velocityUnits::pct);
            back_right_motor.setVelocity(turning, velocityUnits::pct);
            front_left_motor.spin(directionType::fwd);
            front_right_motor.spin(directionType::fwd);
            back_left_motor.spin(directionType::fwd);
            back_right_motor.spin(directionType::fwd);
          }
        

          
        } else if ( con1.Axis1.position() != 0  || con1.Axis2.position() != 0  || con1.Axis3.position() != 0  || con1.Axis4.position() != 0 || con1.ButtonDown.pressing() ) {
          Drives::fieldOriented();
        } else {
          Drives::robotOriented();
        }
      }
      
      if ( con1.ButtonR1.pressing() || con2.ButtonR1.pressing()) {
        DigitalOutA.set(true);
      } else {
        DigitalOutA.set(false);
      }

      if ( con1.ButtonA.pressing() ){
        DigitalOutH.set(true);
      } else {
        DigitalOutH.set(false);
      }

      /// flywheel spionup joko jasjdklfj lsd.
      if (con1.ButtonR2.pressing() || con2.ButtonR2.pressing()) {
        lastSpeed = speed;
        if ( goalcol != 2 ) {
          speed = 0.0028*(VisionSensor.largestObject.width-153.886)*(VisionSensor.largestObject.width-153.886)+28.544;
        }
          
        int runAt = 0;
        if ( VisionSensor.largestObject.width > 95 ) {
          runAt = 11;
        } else if ( VisionSensor.largestObject.width > 80 ) {
          runAt = 9;
        } else if ( VisionSensor.largestObject.width > 50 ) {
          runAt = 9;
        } else if ( VisionSensor.largestObject.width > 35) {
          runAt = 10;
        } else {
          runAt = 11;
        }
        
        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        // what we are actually running at
        con1.Screen.print(VisionSensor.largestObject.width);
        con1.Screen.setCursor(2, 1);
        // what we are telling the motors to run at (PID to correct)
        con1.Screen.print(runAt);
        con1.Screen.setCursor(3, 1);
        // what we want to run at
        con1.Screen.print(speed);
        shooter_left.setBrake(coast);
        shooter_right.setBrake(coast);
        // shooter_left.setVelocity(runAt, velocityUnits::pct);
        // shooter_right.setVelocity(runAt, velocityUnits::pct);
        shooter_left.spin(directionType::fwd, runAt, voltageUnits::volt);
        shooter_right.spin(directionType::fwd, runAt, voltageUnits::volt);
        //task::sleep(100);
        // if ( con1.ButtonR1.pressing() ) {
        //   DigitalOutA.set(false);
        // } else {
        //   DigitalOutA.set(true);
        // }
        
      } else {
        //      lastSpeed = speed;
        //      speed = 0.0028*(VisionSensor.largestObject.width-153.886)*(VisionSensor.largestObject.width-153.886)+28.544;
        //      double percentSpeed = (shooter_left.velocity(velocityUnits::pct) + shooter_right.velocity(velocityUnits::pct))/2;
        //      int runAt = flywheelSpeed.getOutput(percentSpeed, lastSpeed, false, speed) + speed;
        //      shooter_left.setVelocity(0, velocityUnits::pct);
        //      shooter_right.setVelocity(0, velocityUnits::pct);
        //      shooter_left.spin(directionType::fwd);
        //      shooter_right.spin(directionType::fwd);
        lastSpeed = speed;
        speed = 0.0028*(VisionSensor.largestObject.width-153.886)*(VisionSensor.largestObject.width-153.886)+28.544;
        double percentSpeed = (shooter_left.velocity(velocityUnits::pct) + shooter_right.velocity(velocityUnits::pct))/2;
        int runAt = 0;
        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        // what we are actually running at
        con1.Screen.print(percentSpeed);
        con1.Screen.setCursor(2, 1);
        // what we are telling the motors to run at (PID to correct)
        con1.Screen.print(runAt);
        con1.Screen.setCursor(3, 1);
        // what we want to run at
        con1.Screen.print(speed);
        shooter_left.setVelocity(0, velocityUnits::pct);
        shooter_right.setVelocity(0, velocityUnits::pct);
        shooter_left.spin(directionType::fwd);
        shooter_right.spin(directionType::fwd);
      }


      if ( con1.ButtonL1.pressing() || con2.ButtonL1.pressing() ) {
        intakeLeft.spin(directionType::fwd);
        intakeRight.spin(directionType::fwd);
      } else if ( con2.ButtonY.pressing() ) {
        intakeLeft.spin(directionType::rev);
      } else if ( con1.ButtonX.pressing() ) {
        if ( colorSensor.color() != vex::color::red ) {
          intakeLeft.spin(directionType::rev);
        }
      } else if ( con2.ButtonX.pressing() ) {
        intakeRight.spin(directionType::rev);
      } else {
        intakeLeft.stop();
        intakeRight.stop();
      }

      


      if ( con1.ButtonB.pressing() ) {
        expansion.spin(directionType::fwd);
      } else {
        expansion.stop();
      }
      
      
  }

 
}


 


void pre_auton(void){
  con1.Screen.print("ur bad");
  DigitalOutA.set(false);
  DigitalOutH.set(false);
  Inertial2.setHeading(0.0, degrees);
  Inertial2.setRotation(0.0, degrees);
  Inertial2.startCalibration();
  while (Inertial2.isCalibrating()) { 
      task::sleep(10); 
  }
  Inertial2.setHeading(0.0, degrees);
  Inertial2.setRotation(0.0, degrees);
}



int main(){
  Competition.autonomous(auton);
  Competition.drivercontrol(driving);


  // Run the pre-autonomous function.
  pre_auton();


  // Prevent main from exiting with an infinite loop.
  while (true) {
    // Run these independently of auton and driver tasks
    task::sleep(10); // Wait some time between odometry cycles. Test making it shorter for better position estimates
  }
}