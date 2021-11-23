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

#include "defs.h"
#include "Servo.h"
// #include "string"
//#define Serial1 Serial

MPU6050 mpu;
ServoEasing tiltServo;
AccelStepper stepper(AccelStepper::FULL4WIRE,STEP1,STEP2,STEP3,STEP4);


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
    Serial1.begin(BAUDRATE);
    // Wait for verification string from pi
    while (!Serial1.available()) {}
    
    // Search for verification string
    String val = Serial1.readStringUntil('\n');
    if (val == F("test\n")) {
        Serial1.println(F("testing"));
        testingMode = true;
    } else if (val == F("marco\n")) {
        Serial1.println(F("polo"));
    }

    initMPU();

    configTiltServo();

    
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_ACCEL);


    Serial1.println(F("Configuration successful, entering scanning mode"));
}

void loop() {
    
    // If scan flag true and not in testing mode, enter scanning loop
    if (startScanFlag && !testingMode) {
        enterScanningLoop();
        startScanFlag = false;
    }

    // If new data available
    if (newData) {
        strcpy(tempChars, receivedChars);
        parseData(); // Parse movement commands
        newData = false;

        // Update position estimate
        if (updateCurrTiltYaw()) {
            stepper.setCurrentPosition(currTurretYawSteps);
        }

        // Compute absolute target position from relative angles & current angles
        currTargetYawAngleDeg = currTurretYawAngleDeg + relYawDeg;
        currTargetPitchAngleDeg = constrain(currTurretPitchAngleDeg + relPitchDeg,TILT_MIN_ANGLE,TILT_MAX_ANGLE); // Bound pitch to prevent breaking stuff
        if (DO_BOUND_YAW) { // Optionally bound yaw
            currTargetYawAngleDeg = constrain(currTargetYawAngleDeg,-YAW_MAX_WIDTH_DEG/2,YAW_MAX_WIDTH_DEG/2);
        }

        // Update current stepper target
        stepper.moveTo(yawAngle2Steps(currTargetYawAngleDeg));
        // Update current servo target
        tiltServo.startEaseTo(turretAngle2ServoAngle(currTargetPitchAngleDeg));

        lastUpdateTime_ms = millis();
    }

    // Update position estimate
    if (updateCurrTiltYaw()) {
        stepper.setCurrentPosition(currTurretYawSteps);
    }

    stepper.run();

    
    // Read more data from serial buffer
    recvWithStartEndMarkers();

    // If enough time has pased since the last update, reenter scanning mode
    if ((millis() - lastUpdateTime_ms) > SCAN_RESET_TIME_MS)
        startScanFlag = true;

}

void enterScanningLoop() {

    // Reset to 0,0
    stepper.moveTo(0);
    tiltServo.startEaseTo(turretAngle2ServoAngle(0.0));
    while (stepper.distanceToGo() != 0) {
        if (updateCurrTiltYaw()) {
            stepper.setCurrentPosition(currTurretYawSteps);
        }
        
        stepper.run();

        // Read more data from serial buffer
        recvWithStartEndMarkers();
        // Break out if command received from pi while resetting
        if (newData)
            return;
    }

    // Bounce between +-X degrees yaw to scan for targets
    int targetPos = (SCAN_YAW_WIDTH_DEG/2)/360 * STEPS_PER_REV;

    stepper.moveTo(targetPos);
    while (true) {
        if (stepper.distanceToGo() == 0)
            stepper.moveTo(-stepper.currentPosition());

        // Keep stepper position updated by MPU to correct for any missed steps
        if (updateCurrTiltYaw()) {
            stepper.setCurrentPosition(currTurretYawSteps);
        }
        
        stepper.run();

        // Read more data from serial buffer
        recvWithStartEndMarkers();
        // Break out if command received from pi
        if (newData)
            break;
    }
}

int yawAngle2Steps(double yaw) {
    return int(round((yaw/360.0) * double(STEPS_PER_REV)));
}

double servoAngle2TurretAngle(double servoAngleDeg) {
    return PITCH_2_TILTANGLE_OFFSET - convertLinkageAngle(servoAngleDeg + servoAngleOffset,LIFTER1_LEN,servo2TiltAxisLen,LIFTERBASE_2_TILTSHAFT_LEN,LIFTER2_LEN);
}

double turretAngle2ServoAngle(double turretAngleDeg) {
    return servoAngleOffset + convertLinkageAngle(PITCH_2_TILTANGLE_OFFSET-turretAngleDeg,LIFTERBASE_2_TILTSHAFT_LEN,servo2TiltAxisLen,LIFTER1_LEN,LIFTER2_LEN);
}

void setTiltAngle(double turretAngle) {
    tiltServo.writeMicroseconds(tiltAngle2Pulse(turretAngle2ServoAngle(turretAngle)));
    return;
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

void dmpDataReady() {
    mpuInterrupt = true;
}

void initMPU() {
    // Setup mpu6050
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock.

    // initialize device
    Serial1.println(F("Initializing MPU..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial1.println(F("Testing MPU connection..."));
    bool connectionStatus = mpu.testConnection();
    Serial1.println(connectionStatus ? F("MPU6050 connection successful") : F("Error: MPU6050 connection failed"));
    while (!connectionStatus) {}

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Return status code
    Serial1.print(F("DMP Status: "));
    Serial1.println(devStatus);
    if (devStatus != 0) {
        Serial1.println(F("Error: DMP init failed"));
        while (true) {}
    }

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(GYROX_OFFSET);
    mpu.setYGyroOffset(GYROY_OFFSET);
    mpu.setZGyroOffset(GYROZ_OFFSET);
    mpu.setXAccelOffset(ACCELX_OFFSET);
    mpu.setYAccelOffset(ACCELY_OFFSET);
    mpu.setZAccelOffset(ACCELZ_OFFSET);

    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);

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

        //TODO: Ensure that euler angles match up correctly to axes of MPU
        currTurretPitchAngleDeg = ypr[0] * RAD_TO_DEG;
        currTurretYawAngleDeg= ypr[2] * RAD_TO_DEG * -1.0;

        currTurretYawSteps = yawAngle2Steps(currTurretYawAngleDeg);

        return true;
    }
    return false;
}

double findApproxPitch() {
    int numSamples = 1000;
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

//    Serial1.print(sums[0]);
//    Serial1.print(", ");
//    Serial1.println(sums[1]);

    // Use computed accelerations to approximate current turret pitch angle
    return asin(avgY/avgZ)*RAD_TO_DEG;
}

void configTiltServo(){

    Serial1.println(F("Initializing Tilt Servo..."));
    // Setup servo
    tiltServo.attach(SERVO_PIN,TILT_MIN_PULSE,TILT_MAX_PULSE);
    tiltServo.setSpeed(TILT_SPEED_DEG_PER_SEC);
    tiltServo.easeTo(int(round(tiltPulse2Angle(TILT_INIT_PULSE))));
    delay(500); // Wait to stop moving
            
    double approxPitch = findApproxPitch();
    Serial1.print(F("approxPitch: "));
    Serial1.println(approxPitch);

    // Find current servo angle from current pitch
    // Doesn't need to be exact, just need to know approximate starting point to prevent the servo trying to move too far and breaking stuff
    double thetaCS = turretAngle2ServoAngle(approxPitch);
    double currServoAngle = tiltPulse2Angle(TILT_INIT_PULSE);

    servoAngleOffset = thetaCS - currServoAngle;

    // Try to set tilt to zero
    Serial1.println(F("Attempting to zero tilt..."));
    tiltServo.easeTo(int(round(turretAngle2ServoAngle(0.0))));

    delay(500); // Wait to stop moving

    approxPitch = findApproxPitch(); // Recompute approx pitch

    Serial1.print(F("approxPitch: "));
    Serial1.println(approxPitch);

    if (abs(approxPitch) > 1.0) {
        // If approximate pitch close enough to level, reset DMP & exit
        Serial1.print(F("Error: Zeroing unsuccessful, approxPitch = "));
        Serial1.println(approxPitch);
        while (true) {}
    }
    
    // Otherwise, reset DMP & exit
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

    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

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

}
