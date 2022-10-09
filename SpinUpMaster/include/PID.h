#pragma once
class PID {
  double p;
  double i;
  double d;
  double setpoint;
  double cumulativeError;

  public: 
  PID(double inputP, double inputI, double inputD, double inputSetPoint); 
  
  double getOutput(double input, double lastInput, bool readI);

  double getOutput(double input, double lastInput, bool readI, double inputSetpoint);
  
  void setValues(double inputP, double inputI, double inputD);

  void setValues(double inputP, double inputI, double inputD, double inputSetpoint);

  void resetError();

};