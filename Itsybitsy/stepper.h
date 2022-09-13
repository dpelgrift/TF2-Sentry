#include <Arduino.h>
#include "defs.h"
using namespace std;


struct stepper {
    stepper(int p1, int p2, int p3, int p4);

    public:
    void update_config(int32_t steps_per_rev_new, float max_vel_new, float max_accel_new);
    void set_current_rads(double rads);
    void set_rad_target(double target, float feedrate);
    bool step_if_needed();
    double get_current_rads();
    int32_t get_current_steps();
    double get_current_vel();
    int32_t distance_to_go();

    private:
    // Configuration
    uint8_t _pin[4];
    uint16_t _steps_per_rev = YAW_STEPS_PER_REV;

    float _step_size_rads = 2.0 * PI / float(YAW_STEPS_PER_REV);
    float _max_vel = STEPPER_MAX_SPEED * _step_size_rads;
    float _max_accel = STEPPER_ACCEL * _step_size_rads;

    // Backend funcs
    void take_step();
    void step4(long step);
    void set_dir(bool dir);
    void setOutputPins(uint8_t mask);
    void quad_solve(double &t_0, double &t_1, double a, double b, double c);

    // Control vars
    double target_rads = 0;

    // Motion tracking vars
    bool current_dir = false;
    double current_velocity = 0;
    int32_t current_step_count = 0;
    int8_t step_idx = 0;

    double diff_exact_us = 0;
    uint32_t last_step_us = 0;
    uint32_t next_step_us = 0;
};
