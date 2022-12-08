class PID {
  double p;
  double i;
  double d;
  double setpoint;
  double cumulativeError;
  public: 
    PID(double inputP, double inputI, double inputD, double inputSetPoint) {
      p = inputP;
      i = inputI;
      d = inputD;
      setpoint = inputSetPoint;
      cumulativeError = 0;
  }
  
  public:
    double getOutput(double input, double lastInput, bool readI) {
      double error = setpoint - input;
      double lastError = setpoint - lastInput;
      double kp = p * error;
      double ki = 0;
      if ( readI) {
        cumulativeError += error;
        ki = cumulativeError * i;
      }
      double kd = d * ( error - lastError );
      return kp + ki + kd;
  }

  public:
    double getOutput(double input, double lastInput, bool readI, double inputSetpoint) {
      setpoint = inputSetpoint;
      double error = setpoint - input;
      double lastError = setpoint - lastInput;
      double kp = p * error;
      double ki = 0;
      if ( readI) {
        cumulativeError += error;
        ki = cumulativeError * i;
      }
      
      double kd = d * ( error - lastError );
      return kp + ki + kd;
  }
  
  public:
    void setValues(double inputP, double inputI, double inputD) {
      p = inputP;
      i = inputI;
      d = inputD;
  }

  public:
    void setValues(double inputP, double inputI, double inputD, double inputSetpoint) {
      p = inputP;
      i = inputI;
      d = inputD;
      setpoint = inputSetpoint;
  }

  public:
    void resetError() {
      cumulativeError = 0;
  }

};