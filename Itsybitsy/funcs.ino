#include "defs.h"
#include <Arduino.h>
#include <algorithm>
#include <vector>
#include "AccelStepper.h"

bool respondToSerial(char (&serial_data) [MAX_MSG_LEN], uint8_t& index)
{
  if (DataSerial.available() > 0) {
    while (DataSerial.available() > 0) {
      char newchar = DataSerial.read();
      if ((newchar != '\n') and (index < MAX_MSG_LEN)) {
        serial_data[index] = newchar;
        index++;
      }
      else {
        return true;
      }
    }
  }
  return false;
}

void clear_data(char (&serial_data) [MAX_MSG_LEN], uint8_t& index) {
  for (uint16_t i = 0; i < MAX_MSG_LEN; i++) {
    serial_data[i] = '\0';
  }
  index = 0;
}

void parse_inputs(char serial_data[MAX_MSG_LEN], vector<String> &args) {
  char delim = ' ';
  uint32_t index = 0;
  String temp_arg_str = "";

  while (serial_data[index] != '\0') {
    temp_arg_str += serial_data[index];
    index++;
    if (serial_data[index] == delim) {
      args.push_back(temp_arg_str);
      temp_arg_str = "";
      index++;
    }

    // timeout
    if (index > MAX_MSG_LEN) return;
  }
  args.push_back(temp_arg_str);
}

void debug_print_str(String str)
{
  for (uint16_t i = 0; i < str.length(); i++)
  {
    DebugSerial.print(str[i]);
  }
  DebugSerial.println();
}

void parse_int(String inpt, char &cmd, int32_t &value)
{
  cmd = '\0';
  value = NOVALUE;
  cmd = inpt[0];
  String temp_arg_char = "";
  for (uint32_t i = 1; i < inpt.length(); i++)
  {
    temp_arg_char += inpt[i];
  }

  value = temp_arg_char.toFloat();
}

// Reset current stepper internal position to account for any missed steps
void resetStepperPos(AccelStepper& stepper, double yawAngleDeg) {
    long currPos = stepper.currentPosition();

    // Ensure updated position maintains same place in step order (mod(prevPos,4) == mod(newPos,4))
    long currStepIdx = currPos % 4;
    long newStepIdx = yawAngle2Steps(yawAngleDeg) % 4;
    long newPos = yawAngle2Steps(yawAngleDeg) + currStepIdx - newStepIdx;

    long currTarget = stepper.targetPosition();
    float currSpeed = stepper.speed();
    stepper.setCurrentPosition(newPos);
    stepper.setSpeed(currSpeed);
    stepper.moveTo(currTarget);
}

void setStepperParams(AccelStepper& stepper, double newMaxSpeed, double newAccel) {
    if (newMaxSpeed != NOVALUE) stepper.setMaxSpeed(newMaxSpeed);
    if (newAccel != NOVALUE) stepper.setAcceleration(newAccel);
}


// Solve for angles in 4-bar linkage
double convertLinkageAngle(double inputAngleDeg, double A, double B, double C, double D)
{
    double inputAngleRad = inputAngleDeg * PI / 180.0;

    double F = sqrt(pow(A,2) + pow(B,2) - 2.0*A*B*cos(inputAngleRad));

    double thetaOut = acos((pow(B,2) + pow(F,2) - pow(A,2))/(2.0*B*F)) + acos((pow(C,2) + pow(F,2) - pow(D,2))/(2.0*C*F));

    return thetaOut * RAD_TO_DEG;
}

// Convert pwm pulse width to servo angle
double tiltPulse2Angle(long pulse)
{
    long angleLong = map(pulse,TILT_MIN_PULSE,TILT_MAX_PULSE,0,18000);
    return double(angleLong)/100.0;
}

// Convert servo angle to pwm pulse width
long tiltAngle2Pulse(double angle)
{
    long angleLong = long(round(angle*100));
    return map(angleLong,0,18000,TILT_MIN_PULSE,TILT_MAX_PULSE);
}

// Convert yaw angle (in degrees) to number of steps
int yawAngle2Steps(double yaw)
{
    return int(round((yaw/360.0) * double(STEPS_PER_REV)));
}

// Convert # steps to yaw angle (in degrees)
double steps2YawAngle(long steps)
{
    return double(steps)/STEPS_PER_REV * 360.0;
}
