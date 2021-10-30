#include "Arduino.h"
#include "defs.h"

double convertLinkageAngle(double inputAngleDeg, double A, double B, double C, double D) {
    double inputAngleRad = inputAngleDeg * PI / 180.0;

    double F = sqrt(pow(A,2) + pow(B,2) - 2.0*A*B*cos(inputAngleRad));

    double thetaOut = acos((pow(B,2) + pow(F,2) - pow(A,2))/(2.0*B*F)) + acos((pow(C,2) + pow(F,2) - pow(D,2))/(2.0*C*F));

    return thetaOut * RAD_TO_DEG;
}

double tiltPulse2Angle(long pulse) {
    long angleLong = map(pulse,TILT_MIN_PULSE,TILT_MAX_PULSE,0,18000);
    return double(angleLong)/100.0;
}

long tiltAngle2Pulse(double angle) {
    long angleLong = long(round(angle*100));
    return map(angleLong,0,18000,TILT_MIN_PULSE,TILT_MAX_PULSE);
}