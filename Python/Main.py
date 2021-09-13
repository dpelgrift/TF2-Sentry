from CameraDriver import CameraDriver
import SerialDevice
import time
import argparse
import Config as cfg
import subprocess as sp
import CameraDriver



def main():
    # Initialize/verify serial connection
    sd = SerialDevice()
    isVerified = verifySerial(sd)

    if not isVerified:
        print("Serial connection failed")
        while True:
            1


    # Create Camera Handler & Verify Connection
    cam = CameraDriver()
    cam.start()
    # Test video capture by trying to read a frame

    # Create Motor Driver

    # Configure itsybitsy & verify mpu6050 connection

    

    # Zero out tilt

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
