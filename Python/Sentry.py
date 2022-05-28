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
        # Create audio player object & play startup tone
        playSpotSound()

        # Create Motor Driver
        self.motors = MotorDriver()

        # Initialize serial connection
        self.sd = SerialDevice()

        if cfg.BYPASS_SERIAL_VERIFY:
            print("Bypassing serial verification")
        else:
            self.verifySerial()
            
        # Create Camera Handler & Verify Connection
        self.cam = CameraDriver()
        self.cam.start()
        # Test video capture by trying to read a frame
        ret = self.cam.capture.read()
        if not ret:
            self.errorDetected("Camera connection failed")
        print("Camera connection successful")

        self.pitchPid = PID(cfg.kp, cfg.ki, cfg.kd)
        self.yawPid = PID(cfg.kp, cfg.ki, cfg.kd)

    
    def mainLoop(self):
        # Main function loop
        firingTime= None
        isFiring = False
        flyWheelsActive = False
        targetLost = False
        targetLostTime = 0
        scanSoundTime = time.time()
        lastUpdateTime = time.time()
        targetLocked = False
        if cfg.DEBUG_MODE:
            print("Beginning main loop")
        while True:
            # Target search loop
            if not targetLocked:
                
                bbox, frame = self.cam.findTarget() # Constantly look for targets in view
                if bbox is not None: # If target detected
                    if cfg.DISP_FRAME:
                        self.cam.dispTargetFrame(frame,bbox)

                    playSpotSound() # Play spot sound
                    targetLocked = True
                    # self.cam.lockOn(bbox,frame)
                    # self.resetPid()
                    self.motors.flyWheels.on() # Spool up flywheels
                    flyWheelsActive = True
                    targetLost = False
                    if cfg.DEBUG_MODE:
                        print('Target locked on')
                else:
                    # Play scan sound at regular intervals
                    if time.time() - scanSoundTime > cfg.scanSoundPlayInterval:
                        playScanSound()
                        scanSoundTime = time.time()
                    continue

            # h,w = self.cam.getTargetLocation()

            bbox, frame = self.cam.findTarget() # Constantly look for targets in view

            if bbox == None:
                if(isFiring):
                    self.motors.stopFiring()
                    isFiring = False
                    firingTime = None

                if targetLocked and not targetLost:
                    targetLost = True
                    targetLostTime = time.time()

                # If enought time passes without seeing a target, spool down flywheels if they are active & reset lock
                if time.time() - targetLostTime > cfg.spoolDownDelay and targetLocked:
                    targetLostTime = time.time()
                    self.motors.flyWheels.off() # Spool down flywheels
                    flyWheelsActive = False

                    # self.cam.resetLock() # reset lock
                    targetLocked = False
                    continue
            else:
                h = bbox[1] + int(bbox[3] / 2)
                w = bbox[0] + int(bbox[2] / 2)

                pitchPid, yawPid = self.updateTarget(h-cfg.hTargetCenter,w-cfg.wTargetCenter)
                if cfg.DEBUG_MODE:
                    print('PID: {}, {}'.format(pitchPid, yawPid))

                # If target close enough, start firing
                if h-cfg.hTargetCenter < cfg.onTargetPixelProximity and \
                    w-cfg.wTargetCenter < cfg.onTargetPixelProximity:
                    if not isFiring:
                        self.motors.startFiring()
                        isFiring = True
                        firingTime = time.time()
                    else:
                        # Reverse direction of pusher regularly to clear any jams
                        if time.time() - firingTime > cfg.pusherReverseDurationSec:
                            self.motors.reversePusher()
                            firingTime = time.time()
                        
                elif(isFiring):
                    self.motors.stopFiring()
                    isFiring = False
                    firingTime = None

            # If target visible, update position
            # if h != 0 and w != 0:
            #     pitchPid, yawPid = self.updateTarget(h-cfg.hTargetCenter,w-cfg.wTargetCenter)
            #     if cfg.DEBUG_MODE:
            #         print('PID: {}, {}'.format(pitchPid, yawPid))
            # else: # If target not visible
            #     # Reset target lock status so that sentry will immediately look for new targets
            #     self.cam.resetLock()
            #     targetLostTime = time.time()
            #     if cfg.DEBUG_MODE:
            #         print('Resetting lock')

            currTime = time.time()
            if (currTime - lastUpdateTime < cfg.updateRateSec):
                time.sleep(cfg.updateRateSec - (currTime - lastUpdateTime))
                lastUpdateTime = time.time()
                
            
    def updateTarget(self,pitchPixErr,yawPixErr):
        # Convert pixel errors to degree errors
        pitchDegErr = pitchPixErr*cfg.horizFov/cfg.videoResolution[0]
        yawDegErr = yawPixErr*cfg.vertFov/cfg.videoResolution[1]

        if cfg.DEBUG_MODE:
            print('pitchPixErr: {}\tyawPixErr: {}'.format(pitchPixErr, yawPixErr))
            print('pitchDegErr: {}\tyawDegErr: {}'.format(pitchDegErr, yawDegErr))

        if cfg.DISABLE_PID:
            pitchMoveDeg = pitchDegErr
            yawMoveDeg = yawDegErr
        else:
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

        # round vals to keep serial message short. Don't need high precision anyways
        pitchDeg = round(pitchDeg,2)
        yawDeg = round(yawDeg,2)

        # if cfg.DEBUG_MODE:
        #     print('Checkpoint 3a time = {}'.format(time.time()))

        command = '<{},{}>'.format(yawDeg,pitchDeg)
        if cfg.DEBUG_MODE:
            print('Sending Command: {}'.format(command))

        self.sd.command(command)

        # if cfg.DEBUG_MODE:
        #     print('Checkpoint 3b time = {}'.format(time.time()))

    def resetPid(self):
        self.pitchPid.reset()
        self.yawPid.reset()

    def wranglerMode(self):
        # TODO
        print()

    

    def verifySerial(self):
        # Verify serial connection
        if cfg.TEST_MODE:
            isVerified = self.sd.verifySerial('test\n','testing')
        else:
            isVerified = self.sd.verifySerial('marco\n','polo')
        if not isVerified:
            print("Serial connection failed")
            while True:
                1
        print("Serial connection successful")

        # Pass through configuration messages from ItsyBitsy
        while True:
            resp = self.sd.readSerialLine()
            if resp != '':
                print(resp)
            if 'Error' in resp:
                self.errorDetected(resp)
            elif 'entering scanning mode' in resp:
                break

    def errorDetected(self, errMsg):
        playErrorSound()
        print(errMsg)
        print('... exiting in 5 seconds')
        time.sleep(5)
        exit()


if __name__=='__main__':
    sen = Sentry()
    if cfg.TEST_MODE:
        testingMode(sen)
    # TODO: Insert code to check for bluetooth controller to enter wrangler mode
    sen.mainLoop()