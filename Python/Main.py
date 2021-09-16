from CameraDriver import CameraDriver
from SerialDevice import *
import time
import Config as cfg
import subprocess as sp
from CameraDriver import *
from MotorDriver import *
from TestSuite import *


def main():
    # Initialize/verify serial connection
    sd = SerialDevice()

    if cfg.TEST_MODE:
        isVerified = True
    else:
        isVerified = verifySerial(sd)

    if not isVerified:
        print("Serial connection failed")
        while True:
            1
    print("Serial connection successful")

    # Create Camera Handler & Verify Connection
    cam = CameraDriver()
    cam.start()
    # Test video capture by trying to read a frame
    ret = cam.capture.read()
    if not ret:
        print("Camera connection failed")
        while True:
            1
    print("Camera connection successful")

    # Create Motor Driver
    motors = MotorDriver(sd)
    # Configure itsybitsy & verify mpu6050 connection
    #motors.configItsybitsy()

    # Zero out tilt
    motors.zero()

    # If specified, enter testing mode
    if cfg.TEST_MODE:
        testingMode(motors,sd)

    # Enter scanning mode



def verifySerial(sd):
    sd.serial_dev.flush()
    sd.serial_dev.write(('marco\n').encode('ascii'))
    tnow = time.time()
    while time.time() - tnow < sd.R_TIMEOUT:
        if (sd.serial_dev.inWaiting() > 0):  
            resp = sd.serial_dev.readline().decode('ascii')
            if 'polo' in resp:
                return True
            else:
                print("Serial Response:" + resp)
    return False


if __name__=='__main__':
    main()