#include "string.h"
#include "defs.h"
#include "Arduino.h"

double convertLinkageAngle(double inputAngleDeg, double A, double B, double C, double D);

double tiltPulse2Angle(long pulse);

long tiltAngle2Pulse(double angle);