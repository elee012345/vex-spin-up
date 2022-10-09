#include "PID.h"

PID::PID(double inputP, double inputI, double inputD, double inputSetPoint) {
    p = inputP;
    i = inputI;
    d = inputD;
    setpoint = inputSetPoint;
    cumulativeError = 0;
}

double PID::getOutput(double input, double lastInput, bool readI) {
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

double PID::getOutput(double input, double lastInput, bool readI, double inputSetpoint) {
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
    
void PID::setValues(double inputP, double inputI, double inputD) {
    p = inputP;
    i = inputI;
    d = inputD;
}

void PID::setValues(double inputP, double inputI, double inputD, double inputSetpoint) {
    p = inputP;
    i = inputI;
    d = inputD;
    setpoint = inputSetpoint;
}

void PID::resetError() {
    cumulativeError = 0;
}