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
        self.motors = MotorDriver()
        # Configure itsybitsy & verify mpu6050 connection
        self.motors.configItsybitsy()

        # Create audio player object & play startup tone
        self.audio = AudioPlayer()
        self.audio.playSpotSound(self)

        self.pitchPid = PID(cfg.kp, cfg.ki, cfg.kd)
        # self.pitchPid.output_limits = (-0.015, 0.015)
        self.yawPid = PID(cfg.kp, cfg.ki, cfg.kd)
        # self.yawPid.output_limits = (-0.015, 0.015)

    
    def scanningMode(self):
        # Main loop
        doSendScanMessage = True
        firingTime= None
        isFiring = False
        targetLostTime = None
        scanSoundTime = time.time()
        while True:
            # Target search loop
            if not self.cam.targetLocked:
                if doSendScanMessage:
                    self.sd.command('S0')
                    doSendScanMessage = False
                bbox, frame = self.cam.findTarget() # Constantly look for targets in view
                if bbox is None:
                    # Play scan sound at regular intervals
                    if time.time() - scanSoundTime < cfg.scanSoundPlayInterval:
                        self.audio.playScanSound()
                        scanSoundTime = time.time()
                    continue
                else:
                    self.audio.playSpotSound(self) # Play spot sound
                    self.cam.lockOn(self,bbox,frame)
                    self.resetPid()
                    self.motors.flyWheels.on() # Spool up flywheels
                    if cfg.DEBUG_MODE:
                        print('Target locked on')

            h,w = self.cam.getTargetLocation()

            # If target close enough, start firing
            if h-cfg.hTargetCenter < cfg.onTargetPixelProximity and \
                w-cfg.wTargetCenter < cfg.onTargetPixelProximity:
                if isFiring:
                    # Reverse direction of pusher occasionally to clear jams
                    if time.time() - firingTime > cfg.pusherReverseDurationSec:
                        self.motors.reversePusher()
                        firingTime = time.time()
                else:
                    self.motors.beginFiring()
                    isFiring = True
                    firingTime = time.time()
            elif(isFiring):
                self.motors.stopFiring()
                isFiring = False
                firingTime = None


            # If target visible, update position
            if h != 0 and w != 0:
                pitchPid, yawPid = self.updateTarget(h-cfg.hTargetCenter,w-cfg.wTargetCenter)
                if cfg.DEBUG_MODE:
                    print('PID: {}, {}'.format(pitchPid, yawPid))
            else: # If target not visible
                # Reset target lock status so that sentry will immediately look for new target\
                self.cam.resetLock()
                if targetLostTime is not None:
                    targetLostTime = time.time()
                elif targetLostTime > cfg.scanReturnPeriodSec:
                    # Wait some time before re-sending scanning subroutine
                    doSendScanMessage = True
                    targetLostTime = None
                    self.motors.flyWheels.off() # Spool down flywheels
                
            

    def updateTarget(self,pitchPixErr,yawPixErr):
        # Convert pixel errors to degree errors
        pitchDegErr = pitchPixErr*cfg.horizFov/cfg.videoResolution[0]
        yawDegErr = yawPixErr*cfg.vertFov/cfg.videoResolution[1]

        pitchMoveDeg = self.pitchPid(pitchDegErr)
        yawMoveDeg = self.yawPid(yawDegErr)

        self.relMove(pitchMoveDeg,yawMoveDeg)

        if cfg.DEBUG_MODE:
            print('PITCH: {}\tcomponents: {}'.format(pitchMoveDeg, self.pitchPid.components))
            print('YAW: {}\tcomponents: {}'.format(yawMoveDeg, self.yawPid.components))

        return pitchMoveDeg, yawMoveDeg

    def relMove(self, pitchDeg, yawDeg):
        # if pitchDeg < cfg.pitchLimitsDeg[0]:
        #     pitchDeg = cfg.pitchLimitsDeg[0]
        # elif pitchDeg > cfg.pitchLimitsDeg[1]:
        #     pitchDeg = cfg.pitchLimitsDeg[1]

        # round vals to keep serial buffer short
        pitchDeg = round(pitchDeg,1)
        yawDeg = round(yawDeg,1)

        command = '<{},{}>'.format(yawDeg,pitchDeg)
        self.sd.command(command)

    def resetPid(self):
        self.pitchPid.reset()
        self.yawPid.reset()

    def getPosition(self):
        print()

    def configItsybitsy(self):
        # TODO
        print()

    def wranglerMode(self):
        # TODO
        print()


if __name__=='__main__':
    sen = Sentry()
    if cfg.TEST_MODE:
        testingMode(sen)
    # TODO: Insert code to check for bluetooth controller to enter wrangler mode
    sen.scanningMode()