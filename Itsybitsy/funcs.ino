#include "defs.h"
#include <Arduino.h>
#include <algorithm>
#include <vector>

bool respondToSerial(char (&serial_data) [MAX_MSG_LEN])
{
  uint8_t index = 0;
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      char newchar = Serial.read();
      if ((newchar != '\n') and (index < MAX_MSG_LEN)) {
        serial_data[index] = newchar;
        index++;
      }
      else {
        break;
      }
    }
    return true;
  }
  return false;
}

void clear_data(char (&serial_data) [MAX_MSG_LEN]) {
  for (uint16_t i = 0; i < MAX_MSG_LEN; i++) {
    serial_data[i] = '\0';
  }
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
    Serial.print(str[i]);
  }
  Serial.println();
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

// Convert yaw angle (in rads) to number of steps
int yawAngle2Steps(double yaw)
{
    return int(round((yaw/360.0) * double(STEPS_PER_REV)));
}