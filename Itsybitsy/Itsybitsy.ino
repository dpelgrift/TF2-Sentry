#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "AccelStepper.h"
#include "ServoEasing.h"
#include "Sentry.ino"

#include "defs.h"

MPU6050 mpu;
ServoEasing tiltServo;
AccelStepper stepperObj(AccelStepper::FULL4WIRE,STEP1,STEP2,STEP3,STEP4);


// Tilt servo vars
const double servo2TiltAxisLen = sqrt(pow(SERVOSHAFT_X,2) + pow(SERVOSHAFT_Y,2));
const double servo2TiltAxisAngleDeg = atan(SERVOSHAFT_Y/SERVOSHAFT_X)*180.0/PI;

double servoAngleOffset = 0; // Angle offset


// MPU Vars
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// General vars
bool testingMode = false;
bool newData = false;
bool startScanFlag = true;
unsigned long lastUpdateTime_ms;
unsigned long lastImuUpdateTime_ms;

char receivedChars[MAX_MSG_LEN];
char tempChars[MAX_MSG_LEN];

double currTurretPitchAngleDeg = 0;
double currTurretYawAngleDeg = 0;
int currTurretYawSteps = 0;

double currTargetPitchAngleDeg;
double currTargetYawAngleDeg;

float relYawDeg;
float relPitchDeg;

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
        testingMode = true;
    } else if (val.startsWith("marco")) {
        DataSerial.println(F("polo"));
    }
    if (DO_PRINT_DEBUG){
        DebugSerial.println(val);
    }

    initMPU();
    lastImuUpdateTime_ms = millis();

    configTiltServo();

    
    stepperObj.setCurrentPosition(0);
    stepperObj.setMaxSpeed(STEPPER_MAX_SPEED);
    stepperObj.setAcceleration(STEPPER_ACCEL);


    DataSerial.println(F("Configuration successful, entering scanning mode"));
}

void loop() {
    
    // If scan flag true and not in testing mode, enter scanning loop
    if (startScanFlag && !testingMode) {
        enterScanningLoop();
        startScanFlag = false;
    }

    // If new target position available
    if (newData) {
        if (DO_PRINT_DEBUG)
            DataSerial.println("Parsing string");

        strcpy(tempChars, receivedChars);
        parseData(); // Parse movement commands
        newData = false;

        // Update position estimate
        if (updateCurrTiltYaw()) {
//            resetPosition();
        }
        
        // Compute absolute target position from relative angles & current angles
        currTargetYawAngleDeg = currTurretYawAngleDeg + relYawDeg;
        currTargetPitchAngleDeg = constrain(currTurretPitchAngleDeg + relPitchDeg,TILT_MIN_ANGLE,TILT_MAX_ANGLE); // Bound pitch to prevent breaking stuff
        if (DO_BOUND_YAW) { // Optionally bound yaw
            currTargetYawAngleDeg = constrain(currTargetYawAngleDeg,-YAW_MAX_WIDTH_DEG/2,YAW_MAX_WIDTH_DEG/2);
        }

//        DataSerial.print(F("currTargetYawAngleDeg = "));
//        DataSerial.print(currTargetYawAngleDeg);
//        DataSerial.print(F("\tSteps = "));
//        DataSerial.println(yawAngle2Steps(currTargetYawAngleDeg));
//        
//        DataSerial.print(F("currTargetPitchAngleDeg = "));
//        DataSerial.print(currTargetPitchAngleDeg);
//        DataSerial.print(F("\tServo Angle = "));
//        DataSerial.println(turretAngle2ServoAngle(currTargetPitchAngleDeg));

        

        // Update current stepper target
        stepperObj.moveTo(yawAngle2Steps(currTargetYawAngleDeg));
        // Update current servo target
        tiltServo.startEaseTo(turretAngle2ServoAngle(currTargetPitchAngleDeg));

        lastUpdateTime_ms = millis();
    }

    // Update position estimate
    if (updateCurrTiltYaw() && (millis()-lastImuUpdateTime_ms > IMU_UPDATE_DELAY_MS)) {
//        DataSerial.println(F("Turret Pitch/Yaw = "));
//        DataSerial.println(currTurretPitchAngleDeg);
//        DataSerial.println(currTurretYawAngleDeg);

//        resetPosition();
        lastImuUpdateTime_ms = millis();
    }

    // Run stepper forward
    stepperObj.run();

    // Read more data from serial buffer
    recvWithStartEndMarkers();

    // If enough time has pased since the last update, reenter scanning mode
    if ((millis() - lastUpdateTime_ms) > SCAN_RESET_TIME_MS && !testingMode){
        if (DO_PRINT_DEBUG)
            DataSerial.println("Restarting scan");
        startScanFlag = true;
    }
}

void enterScanningLoop() {
    if (DO_PRINT_DEBUG)
        DataSerial.println(F("Entering scanning loop"));

    // Reset to 0,0
    stepperObj.moveTo(0);
    stepperObj.run();
    tiltServo.startEaseTo(turretAngle2ServoAngle(0.0));
    DataSerial.println(F("Resetting zero"));
    while (stepperObj.distanceToGo() != 0) {
        
      
        if (updateCurrTiltYaw()) {
//            stepperObj.setCurrentPosition(currTurretYawSteps);
        }
        
        stepperObj.run();

        // Read more data from serial buffer
        recvWithStartEndMarkers();

        DataSerial.print(F("newData = "));
        DataSerial.println(newData);

        DataSerial.print(F("Distance to go: "));
        DataSerial.println(stepperObj.distanceToGo());
        
        // Break out if command received from pi while resetting
        if (newData){
            DataSerial.println(F("Data received, exiting scan"));
            lastUpdateTime_ms = millis();
            resetPosition();
            return;
        }
    }

    DataSerial.println(F("Finished resetting zero"));

    // Bounce between +-X degrees yaw to scan for targets
    int targetPos = int((SCAN_YAW_WIDTH_DEG/2.0)/360.0 * STEPS_PER_REV);

    DataSerial.print(F("Target Position: "));
    DataSerial.println(targetPos);

    stepperObj.moveTo(targetPos);
    DataSerial.println(F("Entering bounce loop"));
    while (true) {
        
        // Keep stepper position updated by MPU to correct for any missed steps
        if (updateCurrTiltYaw()) {
//            stepperObj.setCurrentPosition(currTurretYawSteps);
        }
        
        stepperObj.run();

        // Read more data from serial buffer
        recvWithStartEndMarkers();

//        DataSerial.print(F("newData = "));
//        DataSerial.println(newData);
//
//        DataSerial.print(F("Distance to go: "));
//        DataSerial.println(stepperObj.distanceToGo());

        
        // Break out if command received from pi
        if (newData){
            if (DO_PRINT_DEBUG)
                DataSerial.println(F("Data received, exiting scan"));
            lastUpdateTime_ms = millis();
            resetPosition();
            break;
        }
        if (stepperObj.distanceToGo() == 0){
            targetPos *= -1;

            if (DO_PRINT_DEBUG)
              DataSerial.println(F("Switching direction"));


            resetPosition();
            stepperObj.moveTo(targetPos);
        }
    }
}

void setTiltAngle(double turretAngle) {
    tiltServo.writeMicroseconds(tiltAngle2Pulse(turretAngle2ServoAngle(turretAngle)));
}

void resetPosition(){
    float currSpeed = stepperObj.speed();
    stepperObj.setCurrentPosition(currTurretYawSteps);
    stepperObj.moveTo(yawAngle2Steps(currTargetYawAngleDeg));
    stepperObj.setSpeed(currSpeed);
}

void dmpDataReady() {
    mpuInterrupt = true;
}

// Convert servo angle to turret angle
double servoAngle2TurretAngle(double servoAngleDeg) {
    double thetaCS = 360 - SERVO_2_TILTSHAFT_ANGLE - servoAngleDeg - servoAngleOffset;
  
    return PITCH_2_TILTANGLE_OFFSET - convertLinkageAngle(thetaCS,LIFTER1_LEN,servo2TiltAxisLen,LIFTERBASE_2_TILTSHAFT_LEN,LIFTER2_LEN);
}

// Convert turret angle to servo angle
double turretAngle2ServoAngle(double turretAngleDeg) {
    double thetaCS = convertLinkageAngle(PITCH_2_TILTANGLE_OFFSET-turretAngleDeg,LIFTERBASE_2_TILTSHAFT_LEN,servo2TiltAxisLen,LIFTER1_LEN,LIFTER2_LEN);

    return 360 - SERVO_2_TILTSHAFT_ANGLE - thetaCS - servoAngleOffset;
}

void initMPU() {
    // Setup mpu6050
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.

    // initialize device
    DataSerial.println(F("Initializing MPU..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    DataSerial.println(F("Testing MPU connection..."));
    bool connectionStatus = mpu.testConnection();
    DataSerial.println(connectionStatus ? F("MPU6050 connection successful") : F("Error: MPU6050 connection failed"));
    while (!connectionStatus) {}

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Return status code
    DataSerial.print(F("DMP Status: "));
    DataSerial.println(devStatus);
    if (devStatus != 0) {
        DataSerial.println(F("Error: DMP init failed"));
        while (true) {}
    }

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(GYROX_OFFSET);
    mpu.setYGyroOffset(GYROY_OFFSET);
    mpu.setZGyroOffset(GYROZ_OFFSET);
    mpu.setXAccelOffset(ACCELX_OFFSET);
    mpu.setYAccelOffset(ACCELY_OFFSET);
    mpu.setZAccelOffset(ACCELZ_OFFSET);

    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}

bool updateCurrTiltYaw() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        currTurretPitchAngleDeg = ypr[2] * RAD_TO_DEG * -1.0;
        currTurretYawAngleDeg= ypr[0] * RAD_TO_DEG;

        if (currTurretYawAngleDeg > 180.0) currTurretYawAngleDeg -= 360.0;
        else if (currTurretYawAngleDeg <= -180.0) currTurretYawAngleDeg += 360.0;

        currTurretYawSteps = yawAngle2Steps(currTurretYawAngleDeg);

        return true;
    }
    return false;
}

double findApproxPitch() {
    int numSamples = 500;
    long sums[] = {0,0};
    double avgY;
    double avgZ;
    VectorInt16 accel;         // [x, y, z]

    // Get average acceleration over many samples
    int i = 1;
    while (i < numSamples) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetAccel(&accel, fifoBuffer);
            sums[0] = sums[0] + accel.y;
            sums[1] = sums[1] + accel.z;

            i++;
        }
    }

    avgY = double(sums[0])/numSamples;
    avgZ = double(sums[1])/numSamples;

    // Use computed accelerations to approximate current turret pitch angle
    return -asin(avgY/avgZ)*RAD_TO_DEG;
}

void configTiltServo(){
    DataSerial.println(F("Initializing Tilt Servo..."));
    
    double approxPitch = findApproxPitch();
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

    approxPitch = findApproxPitch(); // Recompute approx pitch

    DataSerial.print(F("approxPitch: "));
    DataSerial.println(approxPitch);

    if (abs(approxPitch) > 3.0) {
        
        DataSerial.print(F("Error: Zeroing unsuccessful, approxPitch = "));
        DataSerial.println(approxPitch);
        while (true) {}
    }
    
    // If approximate pitch close enough to level, reset DMP & exit
    mpu.resetDMP();
    mpu.resetFIFO();
    mpu.getIntStatus();
    return;
}


// Read serial data into buffer without blocking & detect when end marker received
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

//    if (DO_PRINT_DEBUG)
//        DataSerial.println(F("Checking for new characters"));
//            

    while (DataSerial.available() > 0 && newData == false) {
        rc = DataSerial.read();

//        if (DO_PRINT_DEBUG)
//          DataSerial.println(F("Checking"));

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= MAX_MSG_LEN) {
                    ndx = MAX_MSG_LEN - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                if (DO_PRINT_DEBUG){
                  DataSerial.println(F("End character received"));
                  DataSerial.println(receivedChars);
                }
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

// Parse received target position by splitting the data into its parts
void parseData() { 
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");     // get the yaw
    relYawDeg = atof(strtokIndx);           // convert to a float

    strtokIndx = strtok(NULL, ",");         // get the pitch
    relPitchDeg = atof(strtokIndx);         // convert to a float

    if (DO_PRINT_DEBUG){
        DebugSerial.println(relYawDeg);
        DebugSerial.println(relPitchDeg);
    }
}
