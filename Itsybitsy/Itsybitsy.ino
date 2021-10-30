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
#include "Servo.h"
#include "string"

MPU6050 mpu;
Servo tiltServo;
AccelStepper stepper(AccelStepper::FULL4WIRE,STEP1,STEP2,STEP3,STEP4);

const double servo2TiltAxisLen = sqrt(pow(SERVOSHAFT_X,2) + pow(SERVOSHAFT_Y,2));
const double servo2TiltAxisAngleDeg = atan(SERVOSHAFT_Y/SERVOSHAFT_X)*180.0/PI;

int currTiltPulse = 1500;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


double servoAngleOffset = 0; // Angle offset

void setup() {
    Serial1.begin(BAUDRATE);
    // Wait for verification string from Raspi
    while (!Serial1.available()) {}
    
    // Search for verification string
    String val = Serial1.readStringUntil("\n");
    if (val == F("marco\n")) {
        Serial1.println(F("polo"));
    }

    initMPU();

    uint8_t tiltRet = zeroTilt();

    // If zeroing unsuccessful, send response back to pi
    if (tiltRet == 0) {

    }
    


}

void loop() {

}

long servoAngle2MicroSec(double servoAngle) {
    return map(servoAngle,0,180,TILT_MIN_PULSE,TILT_MAX_PULSE)
}

double servoAngle2TurretAngle(double servoAngleDeg) {
    return PITCH_2_TILTANGLE_OFFSET - convertLinkageAngle(servoAngleDeg + servoAngleOffset,LIFTER1_LEN,servo2TiltAxisLen,LIFTERBASE_2_TILTSHAFT_LEN,LIFTER2_LEN);
}

double turretAngle2ServoAngle(double turretAngleDeg) {
    return servoAngleOffset + convertLinkageAngle(PITCH_2_TILTANGLE_OFFSET-turretAngleDeg,LIFTERBASE_2_TILTSHAFT_LEN,servo2TiltAxisLen,LIFTER1_LEN,LIFTER2_LEN);
}

double convertLinkageAngle(double inputAngleDeg, double A, double B, double C, double D) {
    double inputAngleRad = inputAngleDeg * PI / 180.0;

    double F = sqrt(pow(A,2) + pow(B,2) - 2.0*A*B*cos(inputAngleRad));

    double thetaOut = acos((pow(B,2) + pow(F,2) - pow(A,2))/(2.0*B*F)) + acos((pow(C,2) + pow(F,2) - pow(D,2))/(2.0*C*F));

    return thetaOut * (180.0/PI);
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
    Serial1.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial1.println(F("Testing device connections..."));
    Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Return status code
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

uint8_t zeroTilt(){
    int numSamples = 1000;
    long sums[] = [0,0,0];
    double avgY;
    double avgZ;
    VectorInt16 accel;         // [x, y, z]
    double approxPitch;
    double prevPitch;
    uint8_t retStatus;
    unsigned int currServoPulse = TILT_INIT_PULSE;

    double thetaTS;
    double thetaCS = 0;
    
    // Setup servo
    tiltServo.attach(SERVO_PIN);
    tiltServo.writeMicroseconds(TILT_INIT_PULSE);
    while (true) {
        // Get average acceleration over many samples
        int i = 1;
        while (i < numSamples) {
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                mpu.dmpGetAccel(&accel, fifoBuffer);
                sums[0] = sums[0] + accel.x;
                sums[1] = sums[1] + accel.y;
                sums[2] = sums[2] + accel.z;

                i++;
            }
        }

        avgY = double(sums[1])/numSamples;
        avgZ = double(sums[2])/numSamples;

        // Use computed accelerations to approximate current pitch angle
        approxPitch = asin(avgY/avgZ)*180.0/PI;
        if (abs(approxPitch) < 1.0) {
            // If approximate pitch close enough to level, reset DMP & exit
            mpu.resetDMP();
            mpu.resetFIFO();
            mpu.getIntStatus();
            retStatus = 0;
            break;
        }
        // Otherwise, move closer to level

        // Find current servo angle from current pitch
        thetaTS = PITCH_2_TILTANGLE_OFFSET - approxPitch;

        double thetaCS = turretAngle2ServoAngle(approxPitch);
        

        delay(100);
    }
    return retStatus;
}

