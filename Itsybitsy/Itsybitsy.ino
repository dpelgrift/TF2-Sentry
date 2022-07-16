#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "ServoEasing.h"
#include "stepper.h"
#include "imu.h"
#include "gcode.h"
// #include "funcs.ino"
#include "defs.h"

// Setup hardware objects
stepper step(STEP1,STEP2,STEP3,STEP4);
imu mpu;
ServoEasing tiltServo;


// Tilt servo vars
const double servo2TiltAxisLen = sqrt(pow(SERVOSHAFT_X,2) + pow(SERVOSHAFT_Y,2));
const double servo2TiltAxisAngleDeg = atan(SERVOSHAFT_Y/SERVOSHAFT_X)*180.0/PI;
double servoAngleOffset = 0; // Angle offset


// General vars
unsigned long lastUpdateTime_ms;
unsigned long lastImuUpdateTime_ms;


bool scanningMode = false;
uint8_t scanState = 0;
int scanTargetYaw = SCAN_YAW_WIDTH_DEG/2.0;

unsigned long t0_ms;

char receivedChars[MAX_MSG_LEN];

char base_cmd;
int32_t base_value;

double currTurretPitchAngleDeg = 0;
double currTurretYawAngleDeg = 0;

void setup() {
    delay(2000);
    if (DO_PRINT_DEBUG){
        DebugSerial.begin(BAUDRATE);
        DebugSerial.flush();
        delay(100);
        DebugSerial.println(F("Program start"));
    }

    // Setup servo
    tiltServo.attach(SERVO_PIN,TILT_MIN_PULSE,TILT_MAX_PULSE);
    tiltServo.setEasingType(EASE_QUADRATIC_IN_OUT);
    tiltServo.setSpeed(TILT_SPEED_DEG_PER_SEC);
    int initAngle = int(round(tiltPulse2Angle(TILT_INIT_PULSE)));
    tiltServo.write(initAngle);
    tiltServo.easeTo(initAngle);


    DataSerial.begin(BAUDRATE);
    DataSerial.flush();
    // Search for verification string
    while (DataSerial.available() == 0) {}
    String val = DataSerial.readStringUntil('\n');
    DataSerial.flush();
    if (val.startsWith("test")) {
        delay(500);
        DataSerial.println(F("testing"));
    } else if (val.startsWith("marco")) {
        DataSerial.println(F("polo"));
    }
    if (DO_PRINT_DEBUG){
        DebugSerial.println(val);
    }

    lastImuUpdateTime_ms = millis();

    // Setup stepper with default params
    step.update_config(STEPS_PER_REV,STEPPER_MAX_SPEED,STEPPER_ACCEL);

    DataSerial.println(F("Configuration successful, entering scanning mode"));
    t0_ms = millis();
}

void loop() {

    bool didStep = step.step_if_needed();
    mpu.updateCurrTiltYaw(currTurretPitchAngleDeg,currTurretYawAngleDeg);

    // If in scanning mode, update scan state machine
    if (scanningMode) {
        switch (scanState) {
            case 0: {
                // Reset target to zero yaw, zero pitch
                step.set_current_rads(currTurretYawAngleDeg*DEG_TO_RAD);
                step.set_rad_target(0.0, NOVALUE);
                tiltServo.startEaseTo(turretAngle2ServoAngle(0.0));

                scanState = 1;
                break;
            }
            case 1 : {
                // If reached target, set yaw target to scan width
                if (step.distance_to_go() == 0) {
                    step.set_current_rads(currTurretYawAngleDeg*DEG_TO_RAD);
                    step.set_rad_target(scanTargetYaw, NOVALUE);
                    scanState = 2;
                }
                break;
            }
            case 2 : {
                // If reached target, set yaw target to scan width in other direction
                if (step.distance_to_go() == 0) {
                    step.set_current_rads(currTurretYawAngleDeg*DEG_TO_RAD);
                    step.set_rad_target(-scanTargetYaw, NOVALUE);
                    scanState = 1;
                }
                break;
            }
        }
    }

    if (respondToSerial(receivedChars)) {
        // Parse input into data chunks
        vector<String> args;
        parse_inputs(receivedChars, args);
        parse_int(args[0], base_cmd, base_value);

        switch (tolower(base_cmd)) {
            case 'g': {
                switch (base_value) 
                {
                    case 0: {
                        scanningMode = false;
                        // LINEAR MOVE DO NOT WAIT
                        float xpos, ypos, feedrate;
                        gcode_command_floats gcode(args);
                        if(gcode.com_exists('x'))
                            step.set_rad_target(gcode.fetch('x'), gcode.fetch('f'));
                        if(gcode.com_exists('y'))
                            tiltServo.startEaseTo(turretAngle2ServoAngle(gcode.fetch('y')));
                        break;
                    }
                    case 1: {
                        // Overwrite current pos
                        gcode_command_floats gcode(args);
                        if(gcode.com_exists('x'))
                            step.set_current_rads(gcode.fetch('x'));
                        if(!gcode.com_exists('x'))
                            step.set_current_rads(0);
                        
                        break;
                    }
                    case 2: {
                        // Set scan mode on
                        scanningMode = true;
                        scanState = 0;
                        break;
                    }
                }
                break;
            }

            case 'm': {
                switch (base_value) 
                {
                    case 0: {
                        // Initialize MPU
                        mpu.init();
                        break;
                    }
                    case 1: {
                        // Reset DMP
                        mpu.resetDMP();
                        break;
                    }
                    case 2: {
                        // Configure tilt servo
                        configTiltServo();
                        break;
                    }
                    case 3: {
                        // Get current attitude
                        mpu.updateCurrTiltYaw(currTurretPitchAngleDeg,currTurretYawAngleDeg);
                        float yaw = float(currTurretYawAngleDeg);
                        float tilt = float(currTurretPitchAngleDeg);
                        DataSerial.print(yaw, 4);
                        DataSerial.print(",");
                        DataSerial.print(tilt, 4);
                        DataSerial.print("\n");
                        break;
                    }
                }
                break;
            }
            case 'c': {
                switch (base_value) 
                {
                    case 0: {
                        // Set speed params
                        gcode_command_floats gcode(args);
                        step.update_config(gcode.fetch('a'), gcode.fetch('b'), gcode.fetch('c'));
                        tiltServo.setSpeed(gcode.fetch('d'));
                        break;
                    }
                    case 1: {
                        // Get current yaw velocity
                        float xvel = step.get_current_vel();
                        DataSerial.print(xvel, 4);
                        DataSerial.print("\n");
                        break;
                    }
                }
                break;
            }
            break;
        }
        DataSerial.println("ok");
    }
    
}

// Convert servo angle to turret angle
double servoAngle2TurretAngle(double servoAngleDeg) {
    double thetaCS = 360.0 - SERVO_2_TILTSHAFT_ANGLE - servoAngleDeg - servoAngleOffset;
  
    return PITCH_2_TILTANGLE_OFFSET - convertLinkageAngle(thetaCS,LIFTER1_LEN,servo2TiltAxisLen,LIFTERBASE_2_TILTSHAFT_LEN,LIFTER2_LEN);
}

// Convert turret angle to servo angle
double turretAngle2ServoAngle(double turretAngleDeg) {
    double thetaCS = convertLinkageAngle(PITCH_2_TILTANGLE_OFFSET-turretAngleDeg,LIFTERBASE_2_TILTSHAFT_LEN,servo2TiltAxisLen,LIFTER1_LEN,LIFTER2_LEN);

    return 360.0 - SERVO_2_TILTSHAFT_ANGLE - thetaCS - servoAngleOffset;
}


void configTiltServo(){
    DataSerial.println(F("Initializing Tilt Servo..."));
    
    double approxPitch = mpu.findApproxPitch();
    DataSerial.print(F("approxPitch: "));
    DataSerial.println(approxPitch);

    // Find current servo angle from current pitch
    // Doesn't need to be exact, just need to know approximate starting point to prevent the servo trying to move too far and breaking stuff
    double thetaCS = convertLinkageAngle(PITCH_2_TILTANGLE_OFFSET-approxPitch,LIFTERBASE_2_TILTSHAFT_LEN,servo2TiltAxisLen,LIFTER1_LEN,LIFTER2_LEN);
    double currServoAngle = tiltPulse2Angle(TILT_INIT_PULSE);

    servoAngleOffset = 360.0 - SERVO_2_TILTSHAFT_ANGLE - thetaCS - currServoAngle;

    if (DO_PRINT_DEBUG) {
      DataSerial.print(F("thetaCS: "));
      DataSerial.println(thetaCS);

      DataSerial.print(F("currServoAngle: "));
      DataSerial.println(currServoAngle);

      DataSerial.print(F("servoAngleOffset: "));
      DataSerial.println(servoAngleOffset);

      DataSerial.print(F("zero Angle: "));
      DataSerial.println(((turretAngle2ServoAngle(0.0))));

      DataSerial.print(F("30 deg Angle: "));
      DataSerial.println(((turretAngle2ServoAngle(30.0))));

      DataSerial.print(F("-10 deg Angle: "));
      DataSerial.println(((turretAngle2ServoAngle(-10.0))));
    }

    
    // Try to set tilt to zero
    DataSerial.println(F("Attempting to zero tilt..."));
    tiltServo.easeTo(int(round(turretAngle2ServoAngle(0.0))));

    delay(500); // Wait to stop moving

    approxPitch = mpu.findApproxPitch(); // Recompute approx pitch

    DataSerial.print(F("approxPitch: "));
    DataSerial.println(approxPitch);

    if (abs(approxPitch) > 3.0) {
        
        DataSerial.print(F("Error: Zeroing unsuccessful, approxPitch = "));
        DataSerial.println(approxPitch);
        while (true) {}
    }
    
    // If approximate pitch close enough to level, reset DMP & exit
    mpu.resetDMP();
    return;
}
