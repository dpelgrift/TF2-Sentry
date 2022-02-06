
// Pins
#define SERVO_PIN 5
#define STEP1 9
#define STEP2 10
#define STEP3 12
#define STEP4 13
#define INTERRUPT_PIN 2

// Physical position params
#define SERVOSHAFT_X -75.7
#define SERVOSHAFT_Y -34.0
#define TILTSHAFT_X 0.0
#define TILTSHAFT_Y 0.0
#define LIFTER1_LEN 60.0
#define LIFTER2_LEN 70.0
#define LIFTERBASE_2_TILTSHAFT_LEN 150.3
#define LIFTERBASE_2_TILTSHAFT_RELANGLE_DEG 6.61
#define PITCH_2_TILTANGLE_OFFSET 30.8

// Stepper/Servo params
#define YAW_STEPS_PER_REV 400
#define TILT_MIN_PULSE 500
#define TILT_MAX_PULSE 2500
#define TILT_INIT_PULSE 900
#define TILT_SPEED_DEG_PER_SEC 20

#define STEPS_PER_REV 400
#define STEPPER_MAX_SPEED 150
#define STEPPER_ACCEL 50

#define SCAN_RESET_TIME_MS 5000

// MPU Params
#define ACCELX_OFFSET -2646
#define ACCELY_OFFSET 1617
#define ACCELZ_OFFSET 1306

#define GYROX_OFFSET 67
#define GYROY_OFFSET -29
#define GYROZ_OFFSET 111

//Movement bounds
#define TILT_MIN_ANGLE -10
#define TILT_MAX_ANGLE 30

#define YAW_MAX_WIDTH_DEG 180
#define DO_BOUND_YAW false
#define SCAN_YAW_WIDTH_DEG 120

// Serial params
#define BAUDRATE 115200
#define MAX_MSG_LEN 100

#define DO_PRINT_DEBUG true
#define DebugSerial Serial
