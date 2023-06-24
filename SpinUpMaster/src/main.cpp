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
vex::motor      back_right_motor(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor      back_left_motor(vex::PORT10, vex::gearSetting::ratio18_1, false);
vex::motor      front_right_motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor      front_left_motor(vex::PORT2, vex::gearSetting::ratio18_1, false);

vex::motor      intakeLeft(vex::PORT6, vex::gearSetting::ratio18_1, false);
vex::motor      intakeRight(vex::PORT5, vex::gearSetting::ratio18_1, false);

vex::controller con1(vex::controllerType::primary);
vex::controller con2(vex::controllerType::partner);
vex::inertial   Inertial2(vex::PORT17);
vex::motor      shooter_left(vex::PORT14, vex::gearSetting::ratio18_1, false);
vex::motor      shooter_right(vex::PORT15, vex::gearSetting::ratio18_1, true);

vex::gps        gps(vex::PORT17, 0, turnType::right);
vex::vision     VisionSensor(vex::PORT10);

vex::motor      intake(vex::PORT16, vex::gearSetting::ratio18_1, false);
vex::motor      roller(vex::PORT8, vex::gearSetting::ratio18_1, false);

//vex::motor      expansion(vex::PORT20);
vex::competition Competition;
vex::timer vexTimer;
vex::optical colorSensor(vex::PORT11, false);
vex::timer shootTimer;

vex::rotation left_tracking_wheel(vex::PORT12, true);
vex::rotation right_tracking_wheel(vex::PORT3, true);
vex::rotation center_tracking_wheel(vex::PORT13, false);



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

  public:
    double static getThetaRadians(){
      double deltaL = left_tracking_wheel.position(rev) * pi * twDiameter;
      double deltaR = right_tracking_wheel.position(rev) * pi * twDiameter;

      

      return (deltaL - deltaR)/(tR + tL);

    }

  public:
    double static getThetaDegrees(){
      con1.Screen.clearScreen();
      con1.Screen.setCursor(1, 1);
      con1.Screen.print(getYPosition());
      return std::fmod(getThetaRadians() * 180/pi, 360);
    }

  public:
    double static getYPosition(){
      double theta = Inertial2.angle(deg) * pi/(180);
            double deltaL = left_tracking_wheel.position(rev) * pi * twDiameter;

       return 2 * sin(theta/2) * (deltaL/theta - tL);
    }


  public:
    void static newRobotStrafe(double x, double y){
      //Odometry calculations
      double deltaX = x;
      double deltaY = y;


      double hyp = hypot(deltaX, deltaY);

      double theta = atan(deltaX/deltaY);

      double beta = theta - currHeading*(3.141592/180);


      // degrees for the center tracking wheel to move
      double horizontal = (hyp*sin(beta) / twRadius) * (180/pi) ;

      double vertical = (hyp*cos(beta) / twRadius)  * (180/pi);

      left_tracking_wheel.resetPosition();
      right_tracking_wheel.resetPosition();
      center_tracking_wheel.resetPosition();

      
      // double preVertError = vertical - left_tracking_wheel.position(deg);
      // double preHoriError = horizontal - center_tracking_wheel.position(deg);
      // double yIntegral = 0;
      // double xIntegral = 0;

      front_left_motor.resetRotation();
      back_left_motor.resetRotation();
      front_right_motor.resetRotation();
      back_right_motor.resetRotation();


      double vertError = vertical - left_tracking_wheel.position(deg);
      double horiError = horizontal - center_tracking_wheel.position(deg);



      //PID loop
      while(vertError != 0 || horiError != 0){
        vertError = vertical + left_tracking_wheel.position(deg);
        horiError = horizontal - center_tracking_wheel.position(deg);


        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print(horiError);
        con1.Screen.setCursor(2, 1);
        con1.Screen.print(vertError);

        
        // double xDerivative = horiError - preHoriError;
        // double yDerivative = vertError - preVertError;

        // if(vertError==0 || vertError > 500){
        //   yIntegral = 0;
        // }
        // if(horiError == 0 || horiError >500){
        //   xIntegral = 0;
        // }



        // double kP = 1;
        // double kD = 0;
        // double kI = 0;

        // preVertError = vertError;
        // preHoriError = horiError;

        // apply pid to thingies



        
        Drives::robotOrientedforAuton(horiError/4, vertError/4, 0);




        // different direction where the wheels are pointing so different degrees to turn
        // double front_left_degrees = horiError + vertError;
        // double front_right_degrees = horiError - vertError;
        // double back_left_degrees = horiError - vertError;
        // double back_right_degrees = horiError + vertError;


        // // set turning thingy here
        // front_left_motor.spin(fwd, front_left_degrees, dps);
        // back_left_motor.spin(fwd, back_left_degrees, dps);
        // front_right_motor.spin(fwd, front_right_degrees, dps);
        // back_right_motor.spin(fwd, back_right_degrees, dps);
        // con1.Screen.clearScreen();
        // con1.Screen.setCursor(1, 1);
        // con1.Screen.print(xDegrees);
        // con1.Screen.setCursor(1, 10);
        // con1.Screen.print(front_right_motor.rotation(deg));
        
      }
      front_left_motor.stop();
      back_left_motor.stop();
      front_right_motor.stop();
      back_right_motor.stop();
      
      

    }
  
  public:
    void static resetTW(){
      left_tracking_wheel.resetPosition();
      right_tracking_wheel.resetPosition();
      center_tracking_wheel.resetPosition();
    }

  //false for vertical
  public:
    void static regularRobotStrafe(double x, double y, double xinitialSpeed, double yinitialSpeed){
            left_tracking_wheel.resetPosition();
      right_tracking_wheel.resetPosition();
      center_tracking_wheel.resetPosition();

      double xsetPoint = x/twRadius * (180/3.141592);
      double xErr = xsetPoint - left_tracking_wheel.position(deg);
      double xprevError = xErr;
      double xintegral = 0;

      double ysetPoint = y/twRadius * (180/3.141592);
      double yErr = ysetPoint - center_tracking_wheel.position(deg);
      double yprevError = yErr;
      double yintegral = 0;
      

      //ku 0.3
      //pu 1.25
      double dn = 0.01;
    
      double xkP = 0.155;
      double xkD = 0.09;
      double xkI = 1.1 * dn;



      double ykP = 0.155;
      double ykD = 0.09;
      double ykI = 1.1 * dn;

      // mod angle in case some idiot put in a random crap number
      int end_angle = Inertial2.heading();
      int amountToTurn;
      double amountToTurnIntegral = 0;
      double aTTp = 1.5;
      double aTTi = 0.1 * dn;

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

        double yerror = ysetPoint - left_tracking_wheel.position(deg);
        if(std::abs(yerror) < 200  && yerror != 0){
          yintegral = yintegral + yerror;
        } else {
          yintegral = 0;
        }
        double yderivative = yerror - yprevError;
        yprevError = yerror;
        double ypower = (yerror*ykP + yderivative * ykD + yintegral*ykI) * yinitialSpeed;

        double xerror = -(xsetPoint - center_tracking_wheel.position(deg));

        if(std::abs(xerror) < 200 && xerror != 0){
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


        Drives::robotOrientedforAuton(xpower, ypower, amountToTurnPower);
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
    DigitalOutA.set(false);
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
      // xToGo and yToGo are both in inches
      // endOrientation is in degrees
      // other one is self-explanatory lol
      void static goToAndTurn( double xToGo, double yToGo, double endOrientation, double timeSeconds ) {
      
        
        // SETUP

        // constants
        double WHEEL_DIAMETER = 4; // inches
        double CIRCUMFERENCE = 3.14159 * WHEEL_DIAMETER; // inches
        double GEAR_RATIO = 1/1;
        int ROBOT_DIAMETER = 19; // diagonal across from wheels (front_right to back_left)
        double ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.14159;

        // calculate moving with method parameters
        // how many rotations each motor should turn (at the same time) to translate xToGo and yToGo
        double x_rotations = (xToGo / CIRCUMFERENCE) * GEAR_RATIO;
        double y_rotations = (yToGo / CIRCUMFERENCE) * GEAR_RATIO;
        // converting rotations to degrees
        double xDegrees = x_rotations * 360 / sqrt(2);
        double yDegrees = y_rotations * 360  / sqrt(2);
        
        front_left_motor.resetRotation();
        back_left_motor.resetRotation();
        front_right_motor.resetRotation();
        back_right_motor.resetRotation();



        // CALCULATE ROTATION
        
        // aka wheel rotations per 360 degrees
        double wheelRotationsToRobotRotation = ROBOT_CIRCUMFERENCE / CIRCUMFERENCE;
        double initialOrientation = Inertial2.heading();

        // should probably implement some better way of calculating this like in my newAbsoluteTurnTo method
        double toTurn = endOrientation - initialOrientation;

        // number of wheel rotations per whole rotation multiplied by the fraction of a whole rotation the parameter
        // tells the robot to turn

        // ^ i have no idea what i was trying to say there 
        // takes the number of rotations each wheel needs to turn to one robot rotation 
        // and then takes the number of rotations we've told the robot to turn
        // multiplies the two
        // (fraction of how far the robot should turn * how much it turns in a rotation)
        double wheelRotations = wheelRotationsToRobotRotation * (toTurn/360);


        double frontLeftEndOrientation = -wheelRotations * 360;
        double frontRightEndOrientation = wheelRotations * 360;
        double backLeftEndOrientation = -wheelRotations  * 360;
        double backRightEndOrientation = wheelRotations  * 360;



        // CALCULATE TRANSLATION

        // different direction where the wheels are pointing so different degrees to turn
        // makes sense if you stare at how the wheels are vectored for a little bit but not too hard to understsand
        double front_left_degrees = xDegrees + yDegrees;
        double front_right_degrees = xDegrees - yDegrees;
        double back_left_degrees = xDegrees - yDegrees;
        double back_right_degrees = xDegrees + yDegrees;

        double front_left_degrees_per_second = front_left_degrees/timeSeconds;
        double front_right_degrees_per_second = front_right_degrees/timeSeconds;
        double back_left_degrees_per_second = back_left_degrees/timeSeconds;
        double back_right_degrees_per_second = back_right_degrees/timeSeconds;

        frontLeftEndOrientation  += front_left_degrees ;
        frontRightEndOrientation += front_right_degrees;
        backLeftEndOrientation   += back_left_degrees  ;
        backRightEndOrientation  += back_right_degrees ;



        vex::timer t;
        t.reset();
        int currentTimeSeconds = 0;
        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print("started");
        int count = 0;
        // while (  front_left_motor.rotation(rotationUnits::deg) < frontLeftEndOrientation && front_right_motor.rotation(rotationUnits::deg) < frontRightEndOrientation && back_left_motor.rotation(rotationUnits::deg) < backLeftEndOrientation && back_right_motor.rotation(rotationUnits::deg) < backRightEndOrientation ) {
        
        // NOTE TO SELF:
        // FIGURE OUT WHEN EACH WHEEL NEEDS TO STOP MOVING
        // WHEN THE ROBOT NEEDS TO STOP
        // SOMETIMES THE END ORIENTATION IS NEGATIVE, SOMETIMES IT'S POSITIVE
        // PID??
        

        // yeah and it falls apart here
        // idk how to combine rotation and translation while still measuring everything at the same time
        // we can rotate and move at the same time infinitely... 
        // but how do we measure how far we've translated? how do we measure how far we need to turn each wheel?
        // we can measure rotation because of the gyro, but idk about translation
        // i was mostly just guessing here
        // let me refer you to fieldOrientedAuton (the method above this one)
        // drive at a specified vector (in arbitrary units) and turns at an arbitrary speed
        // how do we convert these to measureable units?
        while ( true ) {
          double currentHeading = Inertial2.heading();
          double headingRadians = currentHeading * 3.14159/180;
          count += 1;
          con1.Screen.clearScreen();
          con1.Screen.setCursor(1, 1);
          con1.Screen.print(count);
          
          int diff = t.time(timeUnits::sec) - currentTimeSeconds;
          currentTimeSeconds = t.time(timeUnits::sec);
          double orientationToBeAt = initialOrientation + ( currentTimeSeconds / timeSeconds * toTurn );
          // number of degrees the robot needs to turn to be at the orientation at this specific time in the loop
          double currentToGo = orientationToBeAt - currentHeading;
          
          // number of wheel rotations to turn to the currentToGo
          double wheelTurning = wheelRotationsToRobotRotation * currentToGo/360.0;
          // number of wheel degrees to turn to the currentToGo
          double wheelTurningDegrees = wheelTurning * 360.0;

          // convert wheel rotations to speed
          // diff is also 'period'
          // if/else is to avoid division by zero in the first loop
          double degreesPerSecond;
          if ( diff != 0 ) {
            degreesPerSecond = wheelTurningDegrees / diff;
          } else {
            degreesPerSecond = 0;
          }
          

          // calculate strafing instead of driving forward
          double sineHeading = sin(headingRadians);
          double cosHeading = cos(headingRadians);
          double rotatedYPos = xToGo * sineHeading + yToGo * cosHeading;
          double rotatedXPos = xToGo * cosHeading - yToGo * sineHeading;


          x_rotations = (rotatedXPos / CIRCUMFERENCE) * GEAR_RATIO;
          y_rotations = (rotatedYPos / CIRCUMFERENCE) * GEAR_RATIO;
          xDegrees = x_rotations * 360 / sqrt(2) * 1.1;
          yDegrees = y_rotations * 360  / sqrt(2) * 1.1;

          // different direction where the wheels are pointing so different degrees to turn
          front_left_degrees = xDegrees + yDegrees;
          front_right_degrees = xDegrees - yDegrees;
          back_left_degrees = xDegrees - yDegrees;
          back_right_degrees = xDegrees + yDegrees;

          front_left_degrees_per_second = front_left_degrees/timeSeconds;
          front_right_degrees_per_second = front_right_degrees/timeSeconds;
          back_left_degrees_per_second = back_left_degrees/timeSeconds;
          back_right_degrees_per_second = back_right_degrees/timeSeconds;



          front_left_motor.spin( fwd, front_left_degrees_per_second  + degreesPerSecond, dps);
          back_left_motor.spin(  fwd, back_left_degrees_per_second   + degreesPerSecond, dps);
          front_right_motor.spin(fwd, front_right_degrees_per_second - degreesPerSecond, dps);
          back_right_motor.spin( fwd, back_right_degrees_per_second  - degreesPerSecond, dps);
        }

        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print("done");
        con1.Screen.setCursor(2, 1);
        con1.Screen.print(count);
        con1.Screen.setCursor(3, 1);
        con1.Screen.print(frontLeftEndOrientation);
        front_left_motor.stop();
        back_left_motor.stop();
        front_right_motor.stop();
        back_right_motor.stop();



        
    }


  

    public:
    bool static rangeChecker(double curr, double target, double error){
      return std::abs(curr - target) < error;
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
    static void newTurnToAbsolute(int angleToGo, int initialSpeed, double p) {     
      
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

      int i = 0;

      // absolute value in case some idiot put in a negative value
      initialSpeed = abs(initialSpeed);

      // if its tuned right you don't overshoot
      // TUNE IT RIGHT AND DON'T LET IT OVERSHOOT
      // if you do overshoot it usually doesn't hit the exact angle and turns the other way
      while ( end_angle != currentHeading ) {
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

        // if we are super close and not touching we accumulate i to make it turn that extra bit
        if ( abs(amountToTurn) < 3 ) {
          i += 1;
        } else {
          i = 0;
        }

        // finally we actually calculate our speed
        // the choice of dividing by 180 is arbitrary
        // but it seems to work find and give nice p values so yeah
        double speed = initialSpeed * (amountToTurn/180.0) * (p + i);

        con1.Screen.clearScreen();
        con1.Screen.setCursor(1, 1);
        con1.Screen.print(amountToTurn);
        con1.Screen.setCursor(2, 1);
        con1.Screen.print(currentHeading);
        front_left_motor.setVelocity(-speed, velocityUnits::pct);
        back_left_motor.setVelocity(-speed, velocityUnits::pct);
        front_right_motor.setVelocity(speed, velocityUnits::pct);
        back_right_motor.setVelocity(speed, velocityUnits::pct);
        front_left_motor.spin(directionType::fwd);
        back_left_motor.spin(directionType::fwd);
        front_right_motor.spin(directionType::fwd);
        back_right_motor.spin(directionType::fwd);

      }
      // now that you're done reading go back to the testing method and read about the turning while moving thing

      // gives the robot extra time just in case but it doesn't seem to improve anything
      // ONLY BECAUSE IT'S TUNED CORRECTLY

      //vex::timer t;
      //t.reset();
      //t.clear();
      //while ( t.time(sec) < 0.2 ) {
      //  currentHeading = Inertial2.heading();
      //
      //  if ( end_angle - currentHeading > 180 ) {
      //    amountToTurn = (end_angle - currentHeading) - 360;
      //  } else {
      //    amountToTurn = end_angle - currentHeading;
      //  }
      //
      //  int speed = initialSpeed * (amountToTurn/180.0) * p;
      //  con1.Screen.clearScreen();
      //  con1.Screen.setCursor(1, 1);
      //  con1.Screen.print(amountToTurn);
      //  con1.Screen.setCursor(2, 1);
      //  con1.Screen.print(currentHeading);
      //  front_left_motor.setVelocity(-speed, velocityUnits::pct);
      //  back_left_motor.setVelocity(-speed, velocityUnits::pct);
      //  front_right_motor.setVelocity(speed, velocityUnits::pct);
      //  back_right_motor.setVelocity(speed, velocityUnits::pct);
      //  front_left_motor.spin(directionType::fwd);
      //  back_left_motor.spin(directionType::fwd);
      //  front_right_motor.spin(directionType::fwd);
      //  back_right_motor.spin(directionType::fwd);
      //
      //
      //}



    }

  public:
    void static rolllerOn(){
      roller.setVelocity(75, pct);
      roller.spin(directionType::fwd);
    }

  public:
   void static rollerOff(){
     roller.stop();
  }

  public:
    void static intakeOn(){
      intake.setVelocity(100, velocityUnits::pct);
      intake.spin(directionType::fwd);
    }

  public:
    void static intakeOff(){
      intake.stop();
    }

  public:
    void static indexerOn(){
      intake.setVelocity(100, velocityUnits::pct);
      intake.spin(directionType::rev);
    }



    

  
};

//autons start here

void skillsAuton(void){
  //First Roller
  

}



void strafeTesting(void){
  //AutonCommands::newRobotStrafe(0, 10);
  AutonCommands::regularRobotStrafe(-24, -12, 1, 1);
}







 


 


 





void autonExpansionTest(void) {
}

void testing(void) {

  //AutonCommands::newGoTo(12, 0, 20, 0.01);
  // ^ i was trying this 
  // does not work lol
  // probably badly tuned pid but idk
  //AutonCommands::goToAndTurn(12, 0, 90, 1);
  //Drives::fieldOriented();

  // turn to 45, 270, 45, 90, 180, 0 degrees respectively at an initial speed of 50 and a p of 1.9
  // tuned decently but could be a little better, your choice if you want to change it
  // go to the method (control click on it) for more information about it
  // ((DO IT))

  // AutonCommands::newTurnToAbsolute(45, 50, 1.9);
  // AutonCommands::newTurnToAbsolute(270, 50, 1.9);
  // this one sometimes overshoots with a p value of 1.9
  // AutonCommands::newTurnToAbsolute(45, 50, 1.8);
  // AutonCommands::newTurnToAbsolute(90, 50, 1.9);
  // AutonCommands::newTurnToAbsolute(180, 50, 1.9);
  // AutonCommands::newTurnToAbsolute(0, 50, 1.9);

  // a few observations about the robot
  //
  // the inertial sensor has drift towards the direction it's turning
  // so is we turn clockwise a lot, it has drift clockwise and thinks it's turned more clockwise than it is
  // so during skills we want to try to turn both ways equally
  // or we need to line up against the wall during the run to verify the robot's orientation/position
  //
  // secondly, the robot does not turn in place
  // it drifts probably from weight distribution and everything not being exact
  // i have no idea if this happens on the new robot
  // but just something to note

  


  // I also have the goToAndTurn method but it doesn't work too well
  // as you can see above it's commented out... 
  // but you can check it out anyway and i'll add lots of comments explaining it

}




void driverAuton() {




}


void driving(void) { 
  roller.setVelocity(75, pct);



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
   while(true) {
    
    
    AutonCommands::getThetaDegrees();

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
       } else if ( con2.Axis1.position() != 0  || con2.Axis2.position() != 0  || con2.Axis3.position() != 0  || con2.Axis4.position() != 0 ) {
         Drives::robotOriented();
       } else {
         Drives::fieldOriented();
       }
     }
    
     //shoot
     if ( con1.ButtonA.pressing() || con2.ButtonA.pressing()) {
        DigitalOutA.set(true);
        shootTimer.clear();
        shootTimer.reset();
        double start = shootTimer.time(msec);
        while ( shootTimer.time(msec) - start < 500 ) {
          DigitalOutA.set(true);
        }
        DigitalOutA.set(false);

        shootTimer.clear();
        shootTimer.reset();
        // deadband so that you don't accidentally hit it again by holding the button down
        start = shootTimer.time(msec);
        while ( shootTimer.time(msec) - start < 200 ) {

        }

     } else {
       DigitalOutA.set(false);
     }


     // expansion
     if ( con2.ButtonLeft.pressing() || con1.ButtonLeft.pressing() ){
       DigitalOutH.set(true);
     } else {
       DigitalOutH.set(false);
     }

     // spin up flywheel/set flywheel speed
     if ( con2.ButtonUp.pressing() || con2.ButtonRight.pressing() || con2.ButtonDown.pressing() ) {
       int runAt = 0;
       if ( con2.ButtonUp.pressing() ) {
         runAt = 12;
       } else if ( con2.ButtonRight.pressing() ) {
         runAt = 10;
       } else if ( con2.ButtonDown.pressing() ) {
         runAt = 7;
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
       double percentSpeed = (shooter_left.velocity(velocityUnits::pct) + shooter_right.velocity(velocityUnits::pct))/2;
       int runAt = 0;
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


     //intake forward
     if ( con1.ButtonL1.pressing() || con2.ButtonL1.pressing() ) {
          intake.setVelocity(100, velocityUnits::pct);
       intake.spin(directionType::fwd);

     }
      //intake reverse
     else if ( con1.ButtonR1.pressing() || con2.ButtonR1.pressing() ) {
          intake.setVelocity(70, velocityUnits::pct);

       intake.spin(directionType::rev);
     } else {
       intake.stop();

     }

     //rollers
     if ( con1.ButtonX.pressing() || con2.ButtonX.pressing()) {
       roller.spin(directionType::fwd);

      //roller reverse
     } else if(con2.ButtonB.pressing()){
       roller.spin(directionType::rev);
     }  else {
       roller.stop();
     }

     //roller to color
     if ( con1.ButtonB.pressing() ) {
       if ( colorSensor.color() != vex::color::red ) {
         intakeLeft.spin(directionType::rev);
       }
     }
     //outake




    




    
    
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

  Competition.autonomous(strafeTesting);
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

