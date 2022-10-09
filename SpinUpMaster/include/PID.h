#pragma once
class PID {
  double p;
  double i;
  double d;
  double setpoint;
  double cumulativeError;

  public: 
  PID(double inputP, double inputI, double inputD, double inputSetPoint); 
  
  public:
  double getOutput(double input, double lastInput, bool readI);

  public:
  double getOutput(double input, double lastInput, bool readI, double inputSetpoint);
  
  public:
  void setValues(double inputP, double inputI, double inputD);

  public:
  void setValues(double inputP, double inputI, double inputD, double inputSetpoint);

  public:
  void resetError();

};