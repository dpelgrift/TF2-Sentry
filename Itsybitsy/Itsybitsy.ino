#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "AccelStepper.h"

#include "defs.h"
#include "funcs.h"
#include "Servo.h"
#include "string"

MPU6050 mpu;
Servo tiltServo;
AccelStepper stepper(AccelStepper::FULL4WIRE,STEP1,STEP2,STEP3,STEP4);


// Tilt servo vars
const double servo2TiltAxisLen = sqrt(pow(SERVOSHAFT_X,2) + pow(SERVOSHAFT_Y,2));
const double servo2TiltAxisAngleDeg = atan(SERVOSHAFT_Y/SERVOSHAFT_X)*180.0/PI;

int currTiltPulse = TILT_INIT_PULSE; // Current PWM pulse length
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
float euler[3];         // [psi, theta, phi]    Euler angle container


// General vars
bool targetDetected = 0;

void setup() {
    Serial1.begin(BAUDRATE);
    // Wait for verification string from Raspi
    while (!Serial1.available()) {}
    
    // Search for verification string
    String val = Serial1.readStringUntil('\n');
    if (val == F("marco\n")) {
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

}

void scanningLoop() {

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

void dmpDataReady() {
    mpuInterrupt = true;
}

void initMPU() {
    // Setup mpu6050
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

    // initialize device
    Serial1.println(F("Initializing MPU..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial1.println(F("Testing MPU connection..."));
    bool connectionStatus = mpu.testConnection();
    Serial1.println(connectionStatus ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    while (!connectionStatus) {}

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Return status code
    Serial1.println(F("DMP Status:"));
    Serial1.println(devStatus);
    while (devStatus != 0) {}

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(GYROX_OFFSET);
    mpu.setYGyroOffset(GYROY_OFFSET);
    mpu.setZGyroOffset(GYROZ_OFFSET);
    mpu.setXAccelOffset(ACCELX_OFFSET);
    mpu.setYAccelOffset(ACCELY_OFFSET);
    mpu.setZAccelOffset(ACCELZ_OFFSET);

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}

void getCurrTiltYaw(double (&tilt), double (&yaw)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);

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

    // Use computed accelerations to approximate current turret pitch angle
    return asin(avgY/avgZ)*RAD_TO_DEG;
}

void configTiltServo(){

    Serial1.println(F("Initializing Tilt Servo..."));
    // Setup servo
    tiltServo.attach(SERVO_PIN);
    tiltServo.writeMicroseconds(TILT_INIT_PULSE);
    delay(500); // Wait to stop moving
            
    double approxPitch = findApproxPitch();

    // Find current servo angle from current pitch
    double thetaCS = turretAngle2ServoAngle(approxPitch);
    double currServoAngle = tiltPulse2Angle(TILT_INIT_PULSE);

    servoAngleOffset = thetaCS - currServoAngle;

    // Try to set tilt to zero
    Serial1.println(F("Attempting to zero tilt..."));
    currTiltPulse = tiltAngle2Pulse(turretAngle2ServoAngle(0.0));
    tiltServo.writeMicroseconds(currTiltPulse);

    delay(1000); // Wait to stop moving

    approxPitch = findApproxPitch(); // Recompute approx pitch

    if (abs(approxPitch) > 1.0) {
        // If approximate pitch close enough to level, reset DMP & exit
        Serial1.println(F("Zeroing unsuccessful"));
        Serial1.println(approxPitch);
        while (true) {}
    }
    
    // Otherwise, reset DMP & exit
    mpu.resetDMP();
    mpu.resetFIFO();
    mpu.getIntStatus();
    return;
}

