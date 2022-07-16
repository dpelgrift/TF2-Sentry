
//#include "gcode.h"
#include "defs.h"
#include <Arduino.h>
#include <algorithm>
#include <vector>

using namespace std;

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
