#include "ServoEasing.h"
#include "AccelStepper.h"
#include "imu.h"
#include "gcode.h"
#include "defs.h"
//#include "funcs.ino"

// Setup hardware objects
AccelStepper stepper(AccelStepper::FULL4WIRE,STEP1,STEP2,STEP3,STEP4);
imu mpu;
ServoEasing tiltServo;


// Tilt servo vars
const double servo2TiltAxisLen = sqrt(pow(SERVOSHAFT_X,2) + pow(SERVOSHAFT_Y,2));
const double servo2TiltAxisAngleDeg = atan(SERVOSHAFT_Y/SERVOSHAFT_X)*180.0/PI;
double servoAngleOffset = 0; // Angle offset


// General vars
unsigned long lastUpdateTime_ms;
unsigned long lastImuUpdateTime_ms;

bool doConfigTiltFlag = false;

bool scanningMode = false;
uint8_t scanState = 0;
int scanTargetYaw = SCAN_YAW_WIDTH_DEG/2.0;

unsigned long t0_ms;
char receivedChars[MAX_MSG_LEN];
uint8_t rcvIdx = 0;

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

    // Initialize Serial Connection
    DataSerial.begin(BAUDRATE);
    DataSerial.flush();
    // Search for incoming data
    while (DataSerial.available() == 0) {}
    String val = DataSerial.readStringUntil('\n');
    DataSerial.flush();
    DataSerial.println("ok");

    // Setup stepper with default params
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_ACCEL);

    DataSerial.println(F("Configuration successful, entering main loop"));
    t0_ms = millis();
    lastImuUpdateTime_ms = millis();
}

void loop() {

    // Advance stepper
    stepper.run();

    // Update current attitude
    mpu.updateCurrTiltYaw(currTurretPitchAngleDeg,currTurretYawAngleDeg);

    if (doConfigTiltFlag) {
        int r = configTiltServo();
        doConfigTiltFlag = false;
    }

    // If in scanning mode, update scan state machine
    if (scanningMode) {
        switch (scanState) {
            case 0: {
                // Reset target to zero yaw, zero pitch
                resetStepperPos(stepper, currTurretYawAngleDeg);
                stepper.moveTo(0);
                tiltServo.startEaseTo(turretAngle2ServoAngle(0.0));
                lastImuUpdateTime_ms = millis();
                scanState = 1;
                break;
            }
            case 1 : {
                // If reached target, set yaw target to scan width
                if (stepper.distanceToGo() == 0) {
                    resetStepperPos(stepper, currTurretYawAngleDeg);
                    stepper.moveTo(scanTargetYaw);
                    lastImuUpdateTime_ms = millis();
                    scanState = 2;
                }
                break;
            }
            case 2 : {
                // If reached target, set yaw target to scan width in other direction
                if (stepper.distanceToGo() == 0) {
                    resetStepperPos(stepper, currTurretYawAngleDeg);
                    stepper.moveTo(-scanTargetYaw);
                    lastImuUpdateTime_ms = millis();
                    scanState = 1;
                }
                break;
            }
        }
    }

    // Update stepper position from IMU data every so often
    if (lastImuUpdateTime_ms + IMU_UPDATE_DELAY_MS > millis()) {
        resetStepperPos(stepper, currTurretYawAngleDeg);
        lastImuUpdateTime_ms = millis();
    }

    // Respond to serial command
    if (respondToSerial(receivedChars,rcvIdx)) {
        if (DO_PRINT_DEBUG){
          DebugSerial.print("Received: ");
          DebugSerial.println(receivedChars);
        }
        // Parse input into data chunks
        vector<String> args;
        parse_inputs(receivedChars, args);
        parse_int(args[0], base_cmd, base_value);

        clear_data(receivedChars,rcvIdx);

        if (DO_PRINT_DEBUG) {
          DebugSerial.print("Parsing: ");
          debug_print_str(args[0]);
        }

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
                            stepper.moveTo(yawAngle2Steps(gcode.fetch('x')));
                        if(gcode.com_exists('y'))
                            tiltServo.startEaseTo(turretAngle2ServoAngle(gcode.fetch('y')));
                        break;
                    }
                    case 1: {
                        // Overwrite current pos
                        gcode_command_floats gcode(args);
                        if(gcode.com_exists('x'))
                            resetStepperPos(stepper, yawAngle2Steps(gcode.fetch('x')));
                        if(!gcode.com_exists('x'))
                            resetStepperPos(stepper, 0);
                        
                        break;
                    }
                    case 2: {
                        // Set scan mode on
                        scanningMode = true;
                        scanState = 0;
                        break;
                    }
                    case 3: {
                        // Set scan mode off & halt in place
                        scanningMode = false;
                        scanState = 0;
                        stepper.move(0);
                        tiltServo.stop();
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
                        int r = mpu.init();
                        break;
                    }
                    case 1: {
                        // Reset DMP
                        mpu.resetDMP();
                        break;
                    }
                    case 2: {
                        // Configure tilt servo
                        doConfigTiltFlag = true;
                        break;
                    }
                    case 3: {
                        // Set speed params
                        gcode_command_floats gcode(args);
                        setStepperParams(stepper, gcode.fetch('a'), gcode.fetch('b'));
                        tiltServo.setSpeed(gcode.fetch('d'));
                        break;
                    }
                    case 4: {
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
                    case 5: {
                        // Get current yaw velocity
                        float xvel = steps2YawAngle(stepper.speed());
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


int configTiltServo(){
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
       return 0;
    }
    
    // If approximate pitch close enough to level, reset DMP & exit
    mpu.resetDMP();

    DataSerial.print(F("tilt ok"));
    return 1;
}
