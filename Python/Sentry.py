from CameraDriver import CameraDriver
from SerialDevice import *
import time
import Config as cfg
import subprocess as sp
from simple_pid import PID
from CameraDriver import *
from MotorDriver import *
from TestSuite import *
from AudioPlayer import *



class Sentry(object):
    def __init__(self):
        # Initialize/verify serial connection
        self.sd = SerialDevice()

        if cfg.TEST_MODE:
            isVerified = True
            print("Test mode active, bypassing serial verification")
        else:
            isVerified = self.sd.verifySerial()

        if not isVerified:
            print("Serial connection failed")
            while True:
                1
        print("Serial connection successful")

        # Create Camera Handler & Verify Connection
        self.cam = CameraDriver()
        self.cam.start()
        # Test video capture by trying to read a frame
        ret = self.cam.capture.read()
        if not ret:
            print("Camera connection failed")
            while True:
                1
        print("Camera connection successful")

        # Create Motor Driver
        self.motors = MotorDriver(self.sd)
        # Configure itsybitsy & verify mpu6050 connection
        self.motors.configItsybitsy()

        # Create audio player object & play startup tone
        self.audio = AudioPlayer()

        self.pitchPid = PID(cfg.kp, cfg.ki, cfg.kd)
        # self.pitchPid.output_limits = (-0.015, 0.015)
        self.yawPid = PID(cfg.kp, cfg.ki, cfg.kd)
        # self.yawPid.output_limits = (-0.015, 0.015)

    
    def scanningMode(self):
        # Main loop
        scanModeState = False
        firingTime= None
        isFiring = False

        while True:
            
            # Target search loop
            if not self.cam.targetLocked:
                if not scanModeState:
                    #TODO Send command to itsybitsy to enter scanning subroutine
                    scanModeState = True
                bbox, frame = self.cam.findTarget() # Constantly look for targets in view
                if bbox is None:
                    continue
                else:
                    self.cam.lockOn(self,bbox,frame)
                    self.resetPid()
                    self.motors.flyWheels.on() # Spool up flywheels
                    if cfg.DEBUG_MODE:
                        print('FACE locked on')

            h,w = self.cam.getTargetLocation()

            # If target close enough, start firing
            if h-cfg.hTargetCenter < cfg.onTargetPixelProximity and \
                w-cfg.wTargetCenter < cfg.onTargetPixelProximity:
                if isFiring:
                    # Reverse direction of pusher occasionally to clear jams
                    if time.time() - firingTime > cfg.pusherReverseDurationSec:
                        self.motors.reversePusher()
                else:
                    self.motors.beginFiring()
                    isFiring = True
                    firingTime = time.time()
            elif(isFiring):
                self.motors.stopFiring()
                isFiring = False
                firingTime = None


            # If target
            if h != 0 and w != 0:
                pitchPid, yawPid = self.updateTarget(h-cfg.hTargetCenter,w-cfg.wTargetCenter)
                if cfg.DEBUG_MODE:
                    print('PID: {}, {}'.format(pitchPid, yawPid))
            else:
                print()

    
                

    def updateTarget(self,pitchPixErr,yawPixErr):
        
        # Convert pixel errors to degree errors
        pitchDegErr = pitchPixErr*cfg.horizFov/cfg.videoResolution[0]
        yawDegErr = yawPixErr*cfg.vertFov/cfg.videoResolution[1]

        pitchMoveDeg = self.pitchPid(pitchDegErr)
        yawMoveDeg = self.yawPid(yawDegErr)




    def resetPid(self):
        self.pitchPid.reset()
        self.yawPid.reset()

    def wranglerMode(self):
        # TODO
        print()


if __name__=='__main__':
    sen = Sentry()
    if cfg.TEST_MODE:
        testingMode(sen)
    # TODO: Insert code to check for bluetooth controller to enter wrangler mode
    sen.scanningMode()