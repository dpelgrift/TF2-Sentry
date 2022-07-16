//#include <Arduino.h>
#include <algorithm>
#include <vector>


using namespace std;

/* GCODE PARSER STUFF */

// GCODE Struct definition
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
