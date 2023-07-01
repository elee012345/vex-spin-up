// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// DigitalOutA          digital_out   A               
// DigitalOutH          digital_out   H               
// DigitalOutB          digital_out   B               
// Optical13            optical       13              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// DigitalOutA          digital_out   A               
// DigitalOutH          digital_out   H               
// DigitalOutB          digital_out   B               
// Optical13            optical       13              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// DigitalOutA          digital_out   A               
// DigitalOutH          digital_out   H               
// DigitalOutB          digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// DigitalOutA          digital_out   A               
// DigitalOutH          digital_out   H               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// DigitalOutA          digital_out   A               
// DigitalOutB          digital_out   B               
// ---- END VEXCODE CONFIGURED DEVICES ----
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
vex::motor      back_right_motor(vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor      back_left_motor(vex::PORT20, vex::gearSetting::ratio18_1, false);
vex::motor      front_right_motor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor      front_left_motor(vex::PORT19, vex::gearSetting::ratio18_1, false);

vex::motor      intakeLeft(vex::PORT6, vex::gearSetting::ratio18_1, false);
vex::motor      intakeRight(vex::PORT5, vex::gearSetting::ratio18_1, false);

vex::controller con1(vex::controllerType::primary);
vex::controller con2(vex::controllerType::partner);
vex::inertial   Inertial2(vex::PORT6);
vex::motor      shooter_left(vex::PORT11, vex::gearSetting::ratio18_1, false);
vex::motor      shooter_right(vex::PORT2, vex::gearSetting::ratio18_1, true);

vex::gps        gps(vex::PORT17, 0, turnType::right);
vex::vision     VisionSensor(vex::PORT10);

vex::motor      intake(vex::PORT9, vex::gearSetting::ratio18_1, true);
//need both for intake
vex::motor      roller(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::distance   locator(vex::PORT14);

vex::competition Competition;
vex::timer vexTimer;
vex::optical colorSensor(vex::PORT11, false);
vex::timer shootTimer;
vex::color  rollerColor(blue);


vex::rotation left_tracking_wheel(vex::PORT10, true);
vex::rotation right_tracking_wheel(vex::PORT1, true);
vex::rotation center_tracking_wheel(vex::PORT5, false);



int driverGoalTurning = 0;
bool aiming = false;


double currX = 0;
double currY = 0;
double currHeading = 0;
double twRadius = 1.625;
double twPlacement = 6.25;
double twDiameter = 3.25;
double tL = 5.875; // or maybe      or 5.875   or 5.875           
double tR = 6.606; // or maybe 6.55 or 6.75    or 6.625

double pi = 3.14159265358979323846;

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
     xPos *= -1;
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




     int turning = con2.Axis1.position(pct);
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
   void static robotOrientedforAuton(double x, double y, double turning) {
     double maxSpeed = 100;
     double yPos = y;
     if ( yPos < 0 ) {
       yPos = (yPos/10)*(yPos/10) * -1;
     } else {
       yPos = (yPos/10)*(yPos/10);
     }
     double xPos = x;
     if ( xPos < 0 ) {
       xPos = (xPos/10)*(xPos/10) * -1;
     } else {
       xPos = (xPos/10)*(xPos/10);
     }
     //xPos *= -1;
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

    
     //What is the largest sum, or is 100 larger?
     max_raw_sum = std::max(std::abs(front_left),std::max(std::abs(back_left),std::max(std::abs(front_right),std::max(std::abs(back_right),maxSpeed))));
    
     //Scale everything down by the factor that makes the largest only 100, if it was over
     front_left  = front_left  / max_raw_sum * maxSpeed;
     back_left   = back_left   / max_raw_sum * maxSpeed;
     front_right = front_right / max_raw_sum * maxSpeed;
     back_right  = back_right  / max_raw_sum * maxSpeed;

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
    
     //Write the manipulated values out to the motors
 
     front_right_motor.spin(fwd,front_right,velocityUnits::pct);
     back_right_motor.spin(fwd,back_right, velocityUnits::pct);
         front_left_motor.spin(fwd,front_left, velocityUnits::pct);
     back_left_motor.spin(fwd,back_left,  velocityUnits::pct);
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

    if ( con1.ButtonUp.pressing() ) {
       rotatedYPos = 7.5;
       con1.Screen.clearScreen();
     }
     if ( con1.ButtonDown.pressing() ) {
       rotatedYPos = -7.5;
       con1.Screen.clearScreen();
     }
     if ( con1.ButtonLeft.pressing() ) {
       rotatedXPos = -7.5;
     }
     if ( con1.ButtonRight.pressing() ) {
       rotatedXPos = 7.5;
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
    

    if ( !aiming ) {
      driverGoalTurning = con1.Axis1.position(pct);
      double magicNumber;
      if ( abs((int)driverGoalTurning) < 93 ) {
        magicNumber = 1.025;
      } else {
        magicNumber = 1.048;
      }
      if ( driverGoalTurning < 0 ) {
        driverGoalTurning = pow(magicNumber, -driverGoalTurning)-1;
        driverGoalTurning *= -1;
      } else {
        driverGoalTurning = pow(magicNumber, driverGoalTurning)-1;
      }
    }
    


     //Now to consider rotation
     //Naively add the rotational axis
     front_left  = front_left  + driverGoalTurning;
     back_left   = back_left   + driverGoalTurning;
     front_right = front_right - driverGoalTurning;
     back_right  = back_right  - driverGoalTurning;
    
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

   public:
   void static fieldOrientedGoal(int turning) {
     double maxSpeed = 100;
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

    if ( con1.ButtonUp.pressing() ) {
       rotatedYPos = 7.5;
     }
     if ( con1.ButtonDown.pressing() ) {
       rotatedYPos = -7.5;
     }
     if ( con1.ButtonLeft.pressing() ) {
       rotatedXPos = -7.5;
     }
     if ( con1.ButtonRight.pressing() ) {
       rotatedXPos = 7.5;
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
    
     front_left_motor.spin(fwd,front_left, pct);
     back_left_motor.spin(fwd,back_left,  pct);
     front_right_motor.spin(fwd,front_right, pct);
     back_right_motor.spin(fwd,back_right, pct);
   }

};


class AutonCommands {
  //false for vertical
  
  public:
    void static goTo(double x, double y, double xinitialSpeed, double yinitialSpeed, double cutOff){
      vexTimer.clear();
      left_tracking_wheel.resetPosition();
      right_tracking_wheel.resetPosition();
      center_tracking_wheel.resetPosition();

      double xsetPoint = x/twRadius * (180/3.141592);
      double xErr = xsetPoint - center_tracking_wheel.position(deg);
      double xprevError = xErr;
      double xintegral = 0;

      double ysetPoint = y/twRadius * (180/3.141592);
      double yErr = ysetPoint - right_tracking_wheel.position(deg);
      double yprevError = yErr;
      double yintegral = 0;
      


      double dn = 0.01;

//.161
      double xkP = 0.16;
      double xkD = 1.16 * dn;
      double xkI = 1.16 * dn;



      double ykP = 0.16;
      double ykD = 1.16 * dn;
      double ykI = 1.16 * dn;

      // mod angle in case some idiot put in a random crap number
      int end_angle = Inertial2.heading();
      int amountToTurn;
      double amountToTurnIntegral = 0;
      double aTTp = 2;
      double aTTi = 0.2*dn;

      // if its tuned right you don't overshoot
      // TUNE IT RIGHT AND DON'T LET IT OVERSHOOT
      // if you do overshoot it usually doesn't hit the exact angle and turns the other way
      double currentHeading = Inertial2.heading();

      

      while( std::fabs(xprevError) > 10.0 || std::fabs(yprevError) > 10.0 ){
        
        currentHeading = Inertial2.heading();

        // the same math i explained in the big long comment above
        // you can read that
        if ( end_angle - currentHeading > 180 ) {
          amountToTurn = (end_angle - currentHeading) - 360;
        } else if ( currentHeading - end_angle > 180) {
          amountToTurn = 360 - (currentHeading - end_angle);
        } else {
          amountToTurn = end_angle - currentHeading;
        }

        // double yLefterror = ysetPoint - left_tracking_wheel.position(deg);
        // double yRighterror = ysetPoint - right_tracking_wheel.position(deg);
        // double yerror = yLefterror > yRighterror ? yLefterror : yRighterror;
        double yerror = ysetPoint - right_tracking_wheel.position(deg);

        if(std::abs(yerror) < 300  && yerror != 0){
          yintegral = yintegral + yerror;
        } else {
          yintegral = 0;
        }
        double yderivative = yerror - yprevError;
        yprevError = yerror;
        double ypower = (yerror*ykP + yderivative * ykD + yintegral*ykI) * yinitialSpeed;

        double xerror = (xsetPoint - center_tracking_wheel.position(deg));

        if(std::abs(xerror) < 300 && xerror != 0){
        xintegral = xintegral + xerror;
        } else {
          xintegral = 0;
        }
        double derivative = xerror - xprevError;
        xprevError = xerror;
        double xpower = (xerror*xkP + derivative * xkD + xintegral*xkI) * xinitialSpeed;



        if(std::abs(amountToTurn) < 2 && amountToTurn != 0){
          amountToTurnIntegral += amountToTurn;
        } else {
          amountToTurnIntegral = 0;
        }


        double amountToTurnPower = (amountToTurn * aTTp + amountToTurnIntegral * aTTi);

        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print(xerror);
        con1.Screen.setCursor(2, 1);
        con1.Screen.print(yerror);


        Drives::robotOrientedforAuton(xpower, ypower, amountToTurnPower*2);
        if(vexTimer.time(sec) > cutOff){
          break;
        }
      }

      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();


      con1.Screen.clearScreen();
      con1.Screen.setCursor(1, 1);
      con1.Screen.print("done");



    }

  
  public:
    void static rotate(double newHeading){
      double degreesToTurn = newHeading - currHeading;
      double setPoint = (degreesToTurn * twPlacement)/twRadius;

      double prevError = setPoint - left_tracking_wheel.position(deg);
      double integral = 0;

      //ku 0.3
      //pu 1.25
      double kP = 0.18;
      double kD = 0.028125;
      double kI = 0.288;



      while(prevError != 0){
        double error = setPoint - left_tracking_wheel.position(deg);
        integral = integral + error;
        if(error == 0 || error > 500 ){
          integral = 0;
        }
        double derivative = error - prevError;
        prevError = error;
        double power = error*kP + derivative * kD + integral*kI;
        //power = (double) (std::min(abs(power), 100));
        front_left_motor.spin(fwd,power, velocityUnits::pct);
        front_right_motor.spin(fwd,-power,velocityUnits::pct);
        back_right_motor.spin(fwd,-power, velocityUnits::pct);          
        back_left_motor.spin(fwd,power,  velocityUnits::pct);

      }

      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();

      currHeading = currHeading + left_tracking_wheel.position(deg) * twRadius/twPlacement;

    }

  public:
    void static pre_auton(void) {
      //con1.Screen.print("ur bad");
      DigitalOutA.set(true);
      DigitalOutB.set(false);
      DigitalOutH.set(false);
      Inertial2.setHeading(0.0, degrees);
      Inertial2.setRotation(0.0, degrees);
      Inertial2.startCalibration();
      while (Inertial2.isCalibrating()) {
          task::sleep(5);
      }
      Inertial2.setHeading(0.0, degrees);
      Inertial2.setRotation(0.0, degrees);
  }



    

  

  public:
    bool static rangeChecker(double curr, double target, double error){
      return std::abs(curr - target) < error;
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

     intakeLeft.stop();
     intakeRight.stop();



    }


  public:
    // angleToGo: a degree from 0 to 359
    // initialSpeed: a scalar, a magnitude, DOES NOT use direction 
    // p: just the p value duh
    void static turnToAbsolute(int angleToGo) {     

      
      // we have compass angles - a range from 0 to 359
      // the number base is 360
      // turning from any number to any number that doesn't cross the modulo limit is fine (doesn't go counterclockwise from 0 - 359)
      // ex: 0 - 270, 40 - 50, 100 - 60, 350 - 0, 70 - 50
      // 
      // however, notice that some of these angles are not optimized and that if the robot turned the other way then it would turn faster
      // we can't deal with normal cases yet, since they might not have optimized turning
      // first we can focus on passing the modulo boundary
      // how do we detect it?
      // we need to initially calculate which way the robot turns by finding the shortest distance to turn to this angle
      // we'll bias the robot to turn clockwise, but if we turn more than 180 degrees clockwise, we want to turn counterclockwise
      // so, if (endAngle - currentAngle > 180), then we want to turn counterclockwise
      // then we just take the number and subtract 360 from it
      // the logic behind this takes some thought
      // say we turn from 0 to 270
      // 270 - 0 >  180, so we do a different calculation
      // we need to turn -90 degrees, which is (270 - 0) - 360
      // same for 10 - 350, 350 - 10 > 180
      // (350 - 10) - 360 = -20
      // etc.
      // 
      // now we have to handle turning the other way: 359 - 0
      // say we want to turn from 270 to 0
      // 0 - 270 is -270, which is not greater than 180
      // so we check the other way around: initialAngle - curentAngle > 180
      // then we do 360 - (currentAngle - endAngle) for our turning
      // it works trust
      // this is basically just the same thing as before you are a smart cookie you can verify this
      // 
      // if we don't need to do either of these things (our turning is less than or equal to 180 when we turn clockwise between 0-359)
      // we just do endAngle - currentAngle
      // easy enough
      //
      // there are different ways to implement this same thing, but this was the first thing I thought of so yeah


      // mod angle in case some idiot put in a random crap number
      int end_angle = angleToGo % 360;
      int currentHeading = Inertial2.heading();
      int amountToTurn;
              if ( end_angle - currentHeading > 180 ) {
          amountToTurn = (end_angle - currentHeading) - 360;
        } else if ( currentHeading - end_angle > 180) {
          amountToTurn = 360 - (currentHeading - end_angle);
        } else {
          amountToTurn = end_angle - currentHeading;
        }
      int prevError = amountToTurn;


      // absolute value in case some idiot put in a negative value
      double initialSpeed = 100;     


      double dn = 0.01;

      double amountToTurnIntegral = 0;
      double aTTp = 0.5 * dn;
      double aTTd = 0;
      double aTTi = 0;

      // if its tuned right you don't overshoot
      // TUNE IT RIGHT AND DON'T LET IT OVERSHOOT
      // if you do overshoot it usually doesn't hit the exact angle and turns the other way
      while (std::abs(amountToTurn) > 1) {
        currentHeading = Inertial2.heading();

        // the same math i explained in the big long comment above
        // you can read that
        if ( end_angle - currentHeading > 180 ) {
          amountToTurn = (end_angle - currentHeading) - 360;
        } else if ( currentHeading - end_angle > 180) {
          amountToTurn = 360 - (currentHeading - end_angle);
        } else {
          amountToTurn = end_angle - currentHeading;
        }


        if(std::abs(amountToTurn) < 5 && amountToTurn != 0){
        amountToTurnIntegral  += amountToTurn;
        } else {
          amountToTurnIntegral = 0;
        }
        double derivative = amountToTurn - prevError;
        prevError = amountToTurn;


        double speed = (amountToTurn*aTTp + derivative * aTTd + amountToTurnIntegral*aTTi) * initialSpeed;

        // if we are super close and not touching we accumulate i to make it turn that extra bit
        // if ( abs(amountToTurn) < 3 ) {
        //   i += 0.1;
        // } else {
        //   i = 0;
        // }

        // finally we actually calculate our speed
        // the choice of dividing by 180 is arbitrary
        // but it seems to work find and give nice p values so yeah
        

        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print(amountToTurn);
        con1.Screen.setCursor(2, 1);
        con1.Screen.print(end_angle);
        front_left_motor.setVelocity(speed, velocityUnits::pct);
        back_left_motor.setVelocity(speed, velocityUnits::pct);
        front_right_motor.setVelocity(-speed, velocityUnits::pct);
        back_right_motor.setVelocity(-speed, velocityUnits::pct);
        front_left_motor.spin(directionType::fwd);
        back_left_motor.spin(directionType::fwd);
        front_right_motor.spin(directionType::fwd);
        back_right_motor.spin(directionType::fwd);

      }

      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();

      
      con1.Screen.clearScreen();
      con1.Screen.setCursor(1, 1);
      con1.Screen.print("done");



    }

   public:
   static void wait(double secs){
    vexTimer.clear();
       while(vexTimer.time(sec) < secs) {
        
       }
   }


  public:
    void static rollerOn(){
      roller.setBrake(coast);
      roller.setVelocity(100, velocityUnits::pct);
      roller.spin(directionType::rev);
    }

  public:
    void static rollerOff(){
      roller.stop();
    }

  public:
    void static intakeOn(){
      roller.setVelocity(100, velocityUnits::pct);
      intake.setVelocity(100, velocityUnits::pct);
      intake.spin(directionType::fwd);
      roller.spin(directionType::fwd);
    }

  public:
    void static intakeOff(){
      intake.stop();
      roller.stop();
    }

  public:
    void static indexerOn(){
      intake.setVelocity(100, velocityUnits::pct);
      intake.spin(directionType::rev);
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
    static void spinUpFlywheel(int velocity){
        shooter_left.setVelocity(velocity, velocityUnits::pct);
        shooter_right.setVelocity(velocity, velocityUnits::pct);
        shooter_left.spin(directionType::fwd);
        shooter_right.spin(directionType::fwd);
    }

  public:
    void static liftIntake(void){
      DigitalOutB.set(true);
    }

  public:
    void static dropIntake(void){
      DigitalOutB.set(false);
    }

  public:
    void static goToRoller(void){
      int speed = -100;
      Optical13.setLightPower(100, percent);
      Optical13.setLight(ledState::on);
      front_left_motor.setVelocity(speed, velocityUnits::pct);
      back_left_motor.setVelocity(speed, velocityUnits::pct);
      front_right_motor.setVelocity(speed, velocityUnits::pct);
      back_right_motor.setVelocity(speed, velocityUnits::pct);
      while(locator.objectDistance(distanceUnits::mm) > 92){
        front_left_motor.spin(directionType::fwd);
       front_right_motor.spin(directionType::fwd);
       back_left_motor.spin(directionType::fwd);
        back_right_motor.spin(directionType::fwd);
      }
      front_left_motor.stop();
      front_right_motor.stop();
      back_left_motor.stop();
      back_right_motor.stop();
      
    }
  
  public:
    void static getOffRoller(void){
      int speed = 75;
      Optical13.setLightPower(100, percent);
      Optical13.setLight(ledState::on);
      front_left_motor.setVelocity(speed, velocityUnits::pct);
      back_left_motor.setVelocity(speed, velocityUnits::pct);
      front_right_motor.setVelocity(speed, velocityUnits::pct);
      back_right_motor.setVelocity(speed, velocityUnits::pct);
      while(locator.objectDistance(distanceUnits::mm) < 200){
        front_left_motor.spin(directionType::fwd);
       front_right_motor.spin(directionType::fwd);
       back_left_motor.spin(directionType::fwd);
        back_right_motor.spin(directionType::fwd);
      }
      front_left_motor.stop();
      front_right_motor.stop();
      back_left_motor.stop();
      back_right_motor.stop();     
    }



  
  //Modular stuff for skills
  public:
    void static doRoller(void){
      double secondsToComplete = 0.65;
      Optical13.setLightPower(100, percent);
      Optical13.setLight(ledState::on);
      double rollerHue = 250;

      if(Optical13.color() == rollerColor){
         rollerOff();
      }
    }

    public:
      static void shoot(double waitTime, double inbetweenTime, int runAt, int shots){
       
       shooter_left.setBrake(coast);
       shooter_right.setBrake(coast);

       shooter_left.spin(directionType::fwd, runAt, voltageUnits::volt);
       shooter_right.spin(directionType::fwd, runAt, voltageUnits::volt);
      
       vexTimer.clear();
       while(vexTimer.time(sec) < waitTime) {


       }


       for(int i=0; i<shots; i++ ){
         DigitalOutA.set(false);
       vexTimer.clear();
       while(vexTimer.time(sec) < inbetweenTime) {}
       DigitalOutA.set(true);
        vexTimer.clear();
       while(vexTimer.time(sec) < 1) {}
       }

       stopShooters();

   }

   public:
    void static intakeThreeStack(void){
      intakeOn();
      goTo(0, 10, 0.25, 0.25, 4);
      wait(3);
      intakeOff();
    }


  

  public:
    void static starting(double speed, double waitTime){
      Optical13.setLightPower(100, percent);
      Optical13.setLight(ledState::on);
      front_left_motor.setVelocity(speed, velocityUnits::pct);
      back_left_motor.setVelocity(speed, velocityUnits::pct);
      front_right_motor.setVelocity(speed, velocityUnits::pct);
      back_right_motor.setVelocity(speed, velocityUnits::pct);
      front_left_motor.spin(directionType::fwd);
      front_right_motor.spin(directionType::fwd);
      back_left_motor.spin(directionType::fwd);
      back_right_motor.spin(directionType::fwd);
      wait(waitTime);
      front_left_motor.stop();
      front_right_motor.stop();
      back_left_motor.stop();
      back_right_motor.stop();
    }
  
};

//autons start here

void skillsAuton(void){
  //RESET INERTIAL SENSOR
  DigitalOutA.set(true);
  AutonCommands::pre_auton();
  //First Roller
  AutonCommands::rollerOn();
  AutonCommands::goToRoller();
  AutonCommands::doRoller();
  // AutonCommands::wait(0.4);
  // AutonCommands::rollerOff();
  AutonCommands::starting(75, 0.35);
  AutonCommands::getOffRoller();
  //AutonCommands::doRoller();

  // //Second Roller
  AutonCommands::turnToAbsolute(120);
  AutonCommands::spinUpFlywheel(90);
  AutonCommands::intakeOn();
  AutonCommands::goTo(0, -13.5, 1, 1, 1.5);
  AutonCommands::turnToAbsolute(90);
  AutonCommands::goTo(0, -14.5, 1, 0.85, 1.65);
  AutonCommands::intakeOff();
  AutonCommands::rollerOn();
  AutonCommands::turnToAbsolute(90);
  AutonCommands::goToRoller();
  AutonCommands::doRoller();
  // AutonCommands::wait(0.4);
  // AutonCommands::rollerOff();
  AutonCommands::starting(75, 0.5);
  AutonCommands::intakeOn();
  

  // //shoot preloads
  AutonCommands::goTo(-40, 16, 1, 1, 1.25);
  AutonCommands::turnToAbsolute(353);
  AutonCommands::shoot(0.5, 1.2, 13, 4);
  AutonCommands::intakeOff();


  //last resort
  // AutonCommands::turnToAbsolute(0);
  // AutonCommands::goTo(13, -30, 1, 1, 1.25);
  // AutonCommands::turnToAbsolute(45);
  // AutonCommands::expand();

  //only if there is a practice field
  //shoot second three
  AutonCommands::goTo(0, -2, 1, 1, 2);
  AutonCommands::turnToAbsolute(225);
  AutonCommands::intakeOn();
  AutonCommands::spinUpFlywheel(50);
  AutonCommands::goTo(0, -87, 1, 0.2, 3.5);
  AutonCommands::turnToAbsolute(300);
  AutonCommands::shoot(1, .7, 12, 4);
  AutonCommands::intakeOff();




  // new pathing
  AutonCommands::turnToAbsolute(180);
  AutonCommands::goTo(0, -30, 1, 1, 3);
  AutonCommands::goTo(-37, 0, 1, 1, 4);
  AutonCommands::rollerOn();
  AutonCommands::turnToAbsolute(180);
  AutonCommands::goToRoller();
  AutonCommands::doRoller();
  AutonCommands::getOffRoller();

  //fourth roller
  AutonCommands::intakeOn();
  AutonCommands::turnToAbsolute(330);
  AutonCommands::goTo(0, -13.5, 1, 1, 1.5);
  AutonCommands::turnToAbsolute(90);
  AutonCommands::goTo(0, -14.5, 1, 0.85, 1.65);
  AutonCommands::intakeOff();
  AutonCommands::rollerOn();
  AutonCommands::turnToAbsolute(270);
  AutonCommands::goToRoller();
  AutonCommands::doRoller();

  AutonCommands::getOffRoller();
  AutonCommands::goTo(10, 10, 1, 1, 2);
  AutonCommands::turnToAbsolute(225);
  AutonCommands::expand();





  //useless

  // //shoot third three
  // AutonCommands::turnToAbsolute(225);
  // AutonCommands::intakeThreeStack();
  // AutonCommands::spinUpFlywheel(100);
  // AutonCommands::turnToAbsolute(300);
  // AutonCommands::shoot(0.2, .7, 12, 3);

  // //third roller
  // AutonCommands::turnToAbsolute(225);
  // AutonCommands::goTo(0, -25, 1, 1, 1.25);
  // AutonCommands::turnToAbsolute(180);
  // AutonCommands::rollerOn();
  // AutonCommands::starting(100, 2);
  // AutonCommands::rollerOff();
  // AutonCommands::goTo(0, 4, 1, 1.5, 1);


  

//old stuff


  // //Shoot preloads
  // AutonCommands::goTo(11.5, 11.5, 1.3, 1.3);
  // AutonCommands::turnToAbsolute(335);
  // AutonCommands::shoot(0.2, 0.7, 14, 2);

  // //First Three stack
  // AutonCommands::turnToAbsolute(230);
  // AutonCommands::goTo(0, 3, 1, 1);
  // AutonCommands::intakeThreeStack();
  // AutonCommands::spinUpFlywheel(75);
  // AutonCommands::turnToAbsolute(115);
  // AutonCommands::shoot(0.5, 0.5, 12, 3);
  
  // //First triple
  // AutonCommands::turnToAbsolute(250);
  // AutonCommands::intakeOn();
  // // AutonCommands::goTo();
  // // AutonCommands::spinUpFlywheel(50);
  // // AutonCommands::intakeOff();
  // // AutonCommands::turnnToAbsolute();
  // AutonCommands::shoot(0.5, 0.5, 11, 3);

  // //Second Roller
  // AutonCommands::turnToAbsolute(0);
  
  
  



}



void strafeTesting(void){
  AutonCommands::pre_auton();
  AutonCommands::turnToAbsolute(330);
  AutonCommands::turnToAbsolute(45);
}







 


 


 







void driverAuton() {




}


void driving(void) { 
  roller.setVelocity(75, velocityUnits::pct);
    bool isExpanded = false;


   //driverAuton();


   // setting stuff up
   shooter_left.setBrake(coast);
   shooter_right.setBrake(coast);

   
  
   intake.setVelocity(100, velocityUnits::pct);
   left_tracking_wheel.resetPosition();
   right_tracking_wheel.resetPosition();



   int screenCenter = 158;
   PID goal(0.6, 1, 1, 0);
   PID flywheelSpeed(0.4, 0, 0.3, 0);
   int speed = 0;
   int lastSpeed = 0;
   int goalcol = 0;
   DigitalOutA.set(true);
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
     } else {

       // locking onto the goal
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
         if ( (con1.ButtonL2.pressing() || con2.ButtonL2.pressing()) && VisionSensor.largestObject.exists ) {
          
           // get middle of targetted object
           int targetMid = VisionSensor.largestObject.originX + (VisionSensor.largestObject.width / 2);
           // other random crap value is 'feedforward' stfu its messed up
           int error = screenCenter - targetMid;
           goal.setValues(0.17, 0.0, 0, targetMid);
           
          int turning;
           // aiming
           if ( error < 20 ) {
             turning = goal.getOutput(screenCenter, 0, true, targetMid);
           } else {
             goal.resetError();
             turning = goal.getOutput(screenCenter, 0, false, targetMid);
           }
          turning *= 1.5;
           turning += -5;

           Drives::fieldOrientedGoal(turning);

         }
       // actually driving the robot around
       }
        // else if ( con2.Axis1.position() != 0  || con2.Axis2.position() != 0  || con2.Axis3.position() != 0  || con2.Axis4.position() != 0 ) {
        //  Drives::robotOriented();
       //}
        else {
         Drives::fieldOriented();
       }
     }
    
     //shoot
     if ( con1.ButtonR1.pressing() || con2.ButtonR1.pressing()) {
        DigitalOutA.set(false);
        shootTimer.clear();
        shootTimer.reset();
        double start = shootTimer.time(msec);
        while ( shootTimer.time(msec) - start < 1000 ) {
          DigitalOutA.set(false);
        }
        DigitalOutA.set(true);

        shootTimer.clear();
        shootTimer.reset();
        // deadband so that you don't accidentally hit it again by holding the button down
        start = shootTimer.time(msec);
        while ( shootTimer.time(msec) - start < 200 ) {

        }

     } else {
       DigitalOutA.set(true);
     }


     // expansion
     if ( con2.ButtonA.pressing()){
       DigitalOutH.set(true);
     } else {
       DigitalOutH.set(false);
     }

     // spin up flywheel/set flywheel speed
     if ( con2.ButtonUp.pressing() || con2.ButtonRight.pressing() || con2.ButtonDown.pressing() ) {
       int runAt = 0;
       if ( con2.ButtonUp.pressing() ) {
         runAt = 13;
       } else if ( con2.ButtonRight.pressing() ) {
         runAt = 12;
       }
       
        else if ( con2.ButtonDown.pressing() ) {
         runAt = 11;
       }
       shooter_left.spin(directionType::fwd, runAt, voltageUnits::volt);
       shooter_right.spin(directionType::fwd, runAt, voltageUnits::volt);
     /// flywheel spionup joko jasjdklfj lsd.
     } else if (con1.ButtonR2.pressing() || con2.ButtonR2.pressing()) {
       lastSpeed = speed;
       if ( goalcol != 2 ) {
         speed = 0.0028*(VisionSensor.largestObject.width-153.886)*(VisionSensor.largestObject.width-153.886)+28.544;
       }
        
       //variable flywheel speed
       int runAt = 0;
       if ( VisionSensor.largestObject.width > 95 ) {
         runAt = 9;
       } else if ( VisionSensor.largestObject.width > 80 ) {
         runAt = 10;
       } else if ( VisionSensor.largestObject.width > 50 ) {
         runAt = 10;
       } else if ( VisionSensor.largestObject.width > 35) {
         runAt = 10;
       } else if (VisionSensor.largestObject.width > 7) {
         runAt = 11;
       } else {
         runAt = 4;
       }
      
       //   con1.Screen.clearScreen();
       //   con1.Screen.setCursor(1, 1);
       //   // what we are actually running at
       //   con1.Screen.print(VisionSensor.largestObject.width);
       //   con1.Screen.setCursor(2, 1);
       //   // what we are telling the motors to run at (PID to correct)
       //   con1.Screen.print(runAt);
       //   con1.Screen.setCursor(3, 1);
       //   // what we want to run at
       //   con1.Screen.print(speed);
       shooter_left.setBrake(coast);
       shooter_right.setBrake(coast);
       if(runAt != 0){
         shooter_left.spin(directionType::fwd, runAt, voltageUnits::volt);
         shooter_right.spin(directionType::fwd, runAt, voltageUnits::volt);
       }




      
     } else {
       lastSpeed = speed;
       speed = 0.0028*(VisionSensor.largestObject.width-153.886)*(VisionSensor.largestObject.width-153.886)+28.544;
       //  con1.Screen.clearScreen();
       //  con1.Screen.setCursor(1, 1);
       //  // what we are actually running at
       //  con1.Screen.print(percentSpeed);
       //  con1.Screen.setCursor(2, 1);
       //  // what we are telling the motors to run at (PID to correct)
       //  con1.Screen.print(runAt);
       //  con1.Screen.setCursor(3, 1);
       //  // what we want to run at
       //  con1.Screen.print(speed);
       shooter_left.setVelocity(0, velocityUnits::pct);
       shooter_right.setVelocity(0, velocityUnits::pct);
       shooter_left.spin(directionType::fwd);
       shooter_right.spin(directionType::fwd);
     }


     //intake forward(
     if ( con1.ButtonL1.pressing() || con2.ButtonL1.pressing() ) {
        intake.setVelocity(100, velocityUnits::pct);
        roller.setVelocity(100, velocityUnits::pct);
        intake.spin(directionType::fwd);
        roller.spin(directionType::fwd);

     }
      //intake reverse
     else if ( con1.ButtonX.pressing()) {
        intake.setVelocity(70, velocityUnits::pct);
       intake.spin(directionType::rev);

     } else  if ( con2.ButtonX.pressing()) {
       roller.spin(directionType::rev);

      //roller reverse
     } else if(con2.ButtonB.pressing()){
       roller.spin(directionType::rev);
     }  else {
       intake.stop();
       roller.stop();
     }



     //roller to color
     if ( con1.ButtonB.pressing() ) {
       if ( colorSensor.color() != vex::color::red ) {
         intakeLeft.spin(directionType::rev);
       }
     }
     //outake

    

     if(con1.ButtonA.pressing()){
       
       isExpanded = !isExpanded;
      DigitalOutB.set(isExpanded);
      AutonCommands::wait(0.1);
     }

    




    




    
    
 }


}












int main(){

  Inertial2.setHeading(0.0, degrees);
  Inertial2.setRotation(0.0, degrees);
  Inertial2.startCalibration();
  while (Inertial2.isCalibrating()) {
      task::sleep(10);
  }
  Inertial2.setHeading(0.0, degrees);
  Inertial2.setRotation(0.0, degrees);

  Competition.autonomous(skillsAuton);
  // ALL OF MY TESTING THINGS ARE RUN FROM THIS TESTING METHOD INSIDE OF THE DRIVERCONTROL THIHNGHY
  // LOOK HERE
  // DON'T MISS IT
  //' THIS IS WHERE EVERYTHING GETS RUN
  // HI ETHAN
  // THESE ARE UR CONMMENTS
  // ALL FOR U
  // <3
  Competition.drivercontrol(driving);
  //  control click on this ^^^^^^^




  // Run the pre-autonomous function.
  



 // Prevent main from exiting with an infinite loop.
  while (true) {
    // Run these independently of auton and driver tasks
    task::sleep(10); // Wait some time between odometry cycles. Test making it shorter for better position estimates
  }
}


// hi ethan!
// ethan tran that is
// because this message is from mr ethan lee
// start here!
// go to the line that says:
//   Competition.drivercontrol(testing);
// i don't know what line that is because i'm constantly adding/deleting comments and code
// it has a big long comment next to it though so i'm sure you'll find it