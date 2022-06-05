 /* STEPPER STUFF */
#include "defs.h"
#include <Arduino.h>
#include <algorithm>
#include <vector>

using namespace std;

struct stepper {
  stepper(int p1, int p2, int p3, int p4);

  public:
  void enable();
  void disable();
  void update_config(int32_t steps_per_rev_new, float max_vel_new, float max_accel_new);
  void set_current_rads(double rads);
  void set_rad_target(double target, float feedrate);
  bool step_if_needed();
  double get_current_rads();
  double get_current_vel();

  private:
  // Configuration
  uint8_t _pin[4];
  int _steps_per_rev = YAW_STEPS_PER_REV;
  float _max_vel = 5;
  float _max_accel = 5;
  float _step_size_rads = 2.0 * PI / float(YAW_STEPS_PER_REV);

  // Backend funcs
  void take_step();
  void set_dir(bool dir);
  void setOutputPins(uint8_t mask);
  void quad_solve(double &t_0, double &t_1, double a, double b, double c);

  // Control vars
  double target_rads = 0;

  // Motion tracking vars
  bool current_dir = false;
  double current_velocity = 0;
  int32_t current_step_count = 0;
  double diff_exact_us = 0;
  uint32_t last_step_us = 0;
  uint32_t next_step_us = 0;
};

struct gcode_command_floats {
  gcode_command_floats(vector<String> inputs);

  public:
  float fetch(char com_key);
  bool com_exists(char com_key);
  

  private:
  void parse_float(String inpt, char &cmd, float &value);

  vector<char> commands;
  vector<float> values;
};

stepper::stepper(int p1, int p2, int p3, int p4)
{
  _pin[0] = p1;
  _pin[1] = p2;
  _pin[2] = p3;
  _pin[3] = p4;
  pinMode(p1, OUTPUT);
  pinMode(p2, OUTPUT);
  pinMode(p3, OUTPUT);
  pinMode(p4, OUTPUT);
  
  current_velocity = 0;
  current_step_count = 0;
}

void stepper::setOutputPins(uint8_t mask)
{
  uint8_t numpins = 4;
  uint8_t i;
  for (i = 0; i < numpins; i++)
    digitalWrite(_pin[i], (mask & (1 << i)) ? (HIGH) : (LOW));
}

// Public - Update stepper parameters. Feed NOVALUE to not change any particular parameter
void stepper::update_config(int32_t steps_per_rev_new, float max_vel_new, float max_accel_new)
{
  if (max_vel_new != NOVALUE)
    _max_vel = max_vel_new;
  if (max_accel_new != NOVALUE)
    _max_accel = max_accel_new;
  if (steps_per_rev_new != NOVALUE)
  {
    _steps_per_rev = steps_per_rev_new;
    _step_size_rads = 2 * PI / _steps_per_rev;
  }
}

// Public - Overwrite the current position to be any rad value designated
void stepper::set_current_rads(double target)
{
  target = target == NOVALUE ? 0 : target;
  double working_count = target * _steps_per_rev;
  working_count /= (2 * PI);
  working_count += 0.4999; // For the rounding
  current_step_count = (int32_t) working_count;
}

// Public - Update target position in rads. Respects any joint momentum.
void stepper::set_rad_target(double target, float feedrate)
{
  target_rads = target;
  next_step_us = micros();
  if (feedrate != NOVALUE)
    _max_vel = feedrate;
}

// Public - The magic sauce. Tracks motor motion and calculates if a step is needed now to stay on track.
bool stepper::step_if_needed()
{
  uint32_t t_now = micros();
  int32_t step_target = (_steps_per_rev * target_rads);
  step_target /= 2 * PI;

  // Check if motor is in right place already
  if((abs(current_velocity) < 0.001) && (step_target == current_step_count))
    return false;
  else if((abs(current_velocity) < 0.001) && (current_step_count > step_target))
    set_dir(false);
  else if((abs(current_velocity) < 0.001) && (current_step_count < step_target))
    set_dir(true);
  
  if(micros() > next_step_us)
  {
    take_step();

    uint32_t cur_step_us = next_step_us;
    double stop_dist_rads = pow(current_velocity, 2) / (_max_accel);
    double stop_pos_rads = (current_step_count * 2 * PI) / _steps_per_rev;

//    Serial.println(stop_pos_rads);
//    Serial.print("\t stop_dist_rads: ");
//    Serial.print(stop_dist_rads);
//    Serial.print("\t stop_pos_rads: ");
//    Serial.print(stop_pos_rads);
//    Serial.print("\t");

    // This mess determines if we need to slow down
    if((current_dir && ((stop_pos_rads + stop_dist_rads) > target_rads)) || (!current_dir && ((stop_pos_rads - stop_dist_rads) < target_rads)))
    {
      // First check if we are coming to a stop
      if((pow(current_velocity, 2) - 2 * _max_accel * _step_size_rads) < 0)
      {
        // See if we should turn round
        if(current_step_count > step_target)
        {
          next_step_us = (2 * next_step_us) - last_step_us;
          set_dir(false);
        }
        else if(current_step_count < step_target)
        {
          next_step_us = (2 * next_step_us) - last_step_us;
          set_dir(true);
        }
        else
        {
          next_step_us = 4294967294;
          diff_exact_us = 4294967294;
        }
      }

      // Otherwise just decelerate normally
      else
      {
        double t0, t1;
        quad_solve(t0, t1, -_max_accel, current_velocity, _step_size_rads);
        double next_step_temp = 1000000 * min(t0, t1);
        diff_exact_us = next_step_temp;
        next_step_us = (uint32_t) (next_step_temp + 0.5);
        next_step_us += cur_step_us;
      }
    }

    // Otherwise check if we can speed up
    else if(abs(current_velocity) < _max_vel)
    {
      // Quadratic has 2 roots, only use one that results in positive time
      double t0, t1;
      quad_solve(t0, t1, _max_accel, current_velocity, _step_size_rads);
      double next_step_temp = 1000000 * max(t0, t1);
      diff_exact_us = next_step_temp;
      next_step_us = (uint32_t) (next_step_temp + 0.5);
      next_step_us += cur_step_us;
    }

    // Last resort is maintain max speed
    else
    {
      next_step_us += 1000000 * _step_size_rads / _max_vel;
    }

    // Update current motor velocity
    current_velocity = _step_size_rads * 1000000;
    current_velocity /= diff_exact_us;
    current_velocity = current_dir ? current_velocity : -current_velocity;
    current_velocity = abs(current_velocity) < 0.01 ? 0 : current_velocity;
    last_step_us = cur_step_us;
  }
  return true;
}

// Public - Return current position in rads
double stepper::get_current_rads()
{
  return 2 * PI * current_step_count / _steps_per_rev;
}

// Public - Return current velocity in rads/sec
double stepper::get_current_vel()
{
  return current_velocity;
}

// Private - Take single step and update step count if control loop designates to do so
void stepper::take_step()
{

}

// Private - Quadratic equation yo
void stepper::quad_solve(double &t_0, double &t_1, double a, double b, double c)
{
  double temp0 = -abs(b);
  double temp1 = sqrt(pow(b, 2) + 2 * a * c);
  t_0 = (temp0 + temp1) / a;
  t_1 = (temp0 - temp1) / a;
}


/* GCODE PARSER STUFF */

gcode_command_floats::gcode_command_floats(vector<String> inputs)
{
  if (inputs.size() == 1)
    return;

  for(uint16_t arg_i = 1; arg_i < inputs.size(); arg_i++)
  {
    char char_value = '\0';
    float float_value = NOVALUE;
    parse_float(inputs[arg_i], char_value, float_value);

    commands.push_back(tolower(char_value));
    values.push_back(float_value);
  }
}

float gcode_command_floats::fetch(char com_key)
{
  vector<char>::iterator itr = find(commands.begin(), commands.end(), com_key);
  if (itr != commands.cend())
  {
    return values[distance(commands.begin(), itr)];
  }

  return NOVALUE;
}

bool gcode_command_floats::com_exists(char com_key)
{
  vector<char>::iterator itr = find(commands.begin(), commands.end(), com_key);
  if (itr != commands.cend())
  {
    return true;
  }

  return false;
}

void gcode_command_floats::parse_float(String inpt, char &cmd, float &value)
{
  if (inpt.length() > 0)
  {
    cmd = inpt[0];
    if (inpt.length() == 1)
      return;

    String temp_arg_char = "";
    for (uint32_t i = 1; i < inpt.length(); i++)
    {
      temp_arg_char += inpt[i];
    }
  
    value = temp_arg_char.toFloat();
  }
}


// Solve for angles in 4-bar linkage
double convertLinkageAngle(double inputAngleDeg, double A, double B, double C, double D) {
    double inputAngleRad = inputAngleDeg * PI / 180.0;

    double F = sqrt(pow(A,2) + pow(B,2) - 2.0*A*B*cos(inputAngleRad));

    double thetaOut = acos((pow(B,2) + pow(F,2) - pow(A,2))/(2.0*B*F)) + acos((pow(C,2) + pow(F,2) - pow(D,2))/(2.0*C*F));

    return thetaOut * RAD_TO_DEG;
}

// Convert pwm pulse width to servo angle
double tiltPulse2Angle(long pulse) {
    long angleLong = map(pulse,TILT_MIN_PULSE,TILT_MAX_PULSE,0,18000);
    return double(angleLong)/100.0;
}

// Convert servo angle to pwm pulse width
long tiltAngle2Pulse(double angle) {
    long angleLong = long(round(angle*100));
    return map(angleLong,0,18000,TILT_MIN_PULSE,TILT_MAX_PULSE);
}

// Convert yaw angle (in rads) to number of steps
int yawAngle2Steps(double yaw) {
    return int(round((yaw/360.0) * double(STEPS_PER_REV)));
}
