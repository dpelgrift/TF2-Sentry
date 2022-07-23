
// #include "imu.h"
#include "defs.h"
#include <algorithm>
#include <vector>

using namespace std;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// IMU Wrapper Struct


int imu::init() {
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
    if (!connectionStatus) return 0;

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Return status code
    DataSerial.print(F("DMP Status: "));
    DataSerial.println(devStatus);
    if (devStatus != 0) {
        DataSerial.println(F("Error: DMP init failed"));
        return 0;
    }

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

    return 1;
}

bool imu::updateCurrTiltYaw(double& currTurretPitchAngleDeg, double& currTurretYawAngleDeg) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        currTurretPitchAngleDeg = ypr[2] * RAD_TO_DEG * -1.0;
        currTurretYawAngleDeg= ypr[0] * RAD_TO_DEG * -1.0;

        if (currTurretYawAngleDeg > 180.0) currTurretYawAngleDeg -= 360.0;
        else if (currTurretYawAngleDeg <= -180.0) currTurretYawAngleDeg += 360.0;

//        if (DO_PRINT_DEBUG) {
//            DataSerial.print(F("Millis: "));
//            DataSerial.println(millis() - t0_ms);
//
//            DataSerial.print(F("currTurretYawAngleDeg = "));
//            DataSerial.print(currTurretYawAngleDeg);
//            DataSerial.print(F("\tSteps = "));
//            DataSerial.println(yawAngle2Steps(currTurretYawAngleDeg));
//
//            DataSerial.print(F("currTurretPitchAngleDeg = "));
//            DataSerial.print(currTurretPitchAngleDeg);
//            DataSerial.print(F("\tServo Angle = "));
//            DataSerial.println(turretAngle2ServoAngle(currTurretPitchAngleDeg));
//        }
        return true;
    }
    return false;
}

double imu::findApproxPitch() {
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
void imu::resetDMP() {
    mpu.resetDMP();
    mpu.resetFIFO();
    mpu.getIntStatus();
}
