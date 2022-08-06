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

        # Initialize serial connection
        self.sd = SerialDevice()

        # Create Motor Driver
        self.motors = MotorDriver(self.sd)

        if cfg.BYPASS_SERIAL_VERIFY:
            print("Bypassing serial verification")
        else:
            self.verifySerial()

        r1 = self.motors.initMPU()
        r2 = self.motors.configTiltServo()


        cfg.t0 = time.time()
            
        # Create Camera Handler & Verify Connection
        self.cam = CameraDriver()
        self.cam.start()
        # Test video capture by trying to read a frame
        # ret = self.cam.camera.capture()
        ret = self.cam.camera.capture_array()
        if len(ret.flatten()) == 0:
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

        if cfg.DO_CLEAR_IMGS:
            os.system(f'rm -rf {cfg.imgSaveDir}')
            os.system(f'mkdir {cfg.imgSaveDir}')

        if cfg.DO_SCAN:
            self.motors.startScan()

        while True:
            resp = self.sd.readSerialLine()
            while resp != '':
                if resp != '':
                    print(f'T: {time.time()-cfg.t0}, ' + resp)
                resp = self.sd.readSerialLine()

            # Get current attitude
            currYaw,currPitch = self.motors.getCurrYawPitch()

            bbox, frame = self.cam.findTarget() # Constantly look for targets in view

            # If no current target
            if not targetLocked:
                if bbox is not None: # If target detected
                    if cfg.DISP_FRAME:
                        self.cam.dispTargetFrame(frame,bbox)

                    playSpotSound() # Play spot sound
                    # self.cam.lockOn(bbox,frame)
                    # self.resetPid()
                    self.motors.flyWheels.on() # Spool up flywheels
                    targetLost = False
                    targetLocked = True
                    if cfg.DEBUG_MODE:
                        print(f'T: {time.time() - cfg.t0}, Target locked on')
                else: # If still scanning
                    # Play scan sound at regular intervals
                    if time.time() - scanSoundTime > cfg.scanSoundPlayInterval:
                        playScanSound()
                        scanSoundTime = time.time()
                    continue

            # h,w = self.cam.getTargetLocation()
            if bbox == None: # If target not visible
                if isFiring:
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

                    targetLocked = False

                    # Send scan start command
                    if cfg.DO_SCAN:
                        self.motors.startScan()
                    continue
            else:
                targetLost = False
                h = bbox[1] + int(bbox[3] / 2)
                w = bbox[0] + int(bbox[2] / 2)

                pitchErr = cfg.hTargetCenter-h
                yawErr = cfg.wTargetCenter-w

                # Compute absolute target position & send to itsy
                pitchPid, yawPid = self.updateTarget(pitchErr,yawErr,currPitch,currYaw)
                if cfg.DEBUG_MODE:
                    print('PID: {}, {}'.format(pitchPid, yawPid))

                # If target close enough, start firing
                if abs(pitchErr) < cfg.onTargetPixelProximity and \
                    abs(yawErr) < cfg.onTargetPixelProximity:
                    if not isFiring:
                        self.motors.startFiring()
                        isFiring = True
                        firingTime = time.time()
                    else:
                        # Reverse direction of pusher regularly to clear any jams
                        if time.time() - firingTime > cfg.pusherReverseDurationSec:
                            self.motors.reversePusher()
                            firingTime = time.time()
                        
                elif isFiring: # stop firing if too far off
                    self.motors.stopFiring()
                    isFiring = False
                    firingTime = None

            # Delay to limit update rate
            currTime = time.time()
            if (currTime - lastUpdateTime < cfg.updateRateSec):
                waitTime = cfg.updateRateSec - (currTime - lastUpdateTime)
                print(f'T: {time.time() - cfg.t0}, Waiting: {waitTime: .3f} sec')
                time.sleep(cfg.updateRateSec - (currTime - lastUpdateTime))
            
            lastUpdateTime = time.time()

            
    def updateTarget(self,pitchPixErr,yawPixErr,currPitch,currYaw):

        # Convert pixel errors to degree errors
        pitchDegErr = pitchPixErr*cfg.horizFov/cfg.videoResolution[0]
        yawDegErr = yawPixErr*cfg.vertFov/cfg.videoResolution[1]

        # Calc absolute target position
        absTargPitch = currPitch + pitchDegErr
        absTargYaw = currYaw + yawDegErr


        if cfg.DEBUG_MODE:
            t = time.time()-cfg.t0
            print(f'T: {t}, pitchPixErr:\t{pitchPixErr}\tyawPixErr: {yawPixErr}\n')
            print(f'T: {t}, pitchDegErr:\t{pitchDegErr}\tyawDegErr: {yawDegErr}\n')
            print(f'T: {t}, absTargPitch:\t{absTargPitch}\absTargYaw: {absTargYaw}')

        if cfg.DISABLE_PID:
            pitchMoveDeg = absTargPitch
            yawMoveDeg = absTargYaw
        else:
            pitchMoveDeg = self.pitchPid(pitchDegErr)
            yawMoveDeg = self.yawPid(yawDegErr)

        self.absMove(pitchMoveDeg,yawMoveDeg)

        # if cfg.DEBUG_MODE:
        #     print('PITCH: {}\tYAW: {}'.format(pitchMoveDeg, yawMoveDeg))

        return pitchMoveDeg, yawMoveDeg

    def absMove(self, pitchDeg, yawDeg):
        if pitchDeg < cfg.pitchLimitsDeg[0]:
            pitchDeg = cfg.pitchLimitsDeg[0]
        elif pitchDeg > cfg.pitchLimitsDeg[1]:
            pitchDeg = cfg.pitchLimitsDeg[1]

        # round vals to keep serial message short. Don't need high precision anyways
        pitchDeg = round(pitchDeg,2)
        yawDeg = round(yawDeg,2)

        self.motors.move(yawDeg,pitchDeg)

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
            self.errorDetected("Serial connection failed")
        print("Serial connection successful")

    def errorDetected(self, errMsg):
        playErrorSound()
        print(errMsg)
        print('... exiting in 5 seconds')
        time.sleep(5)
        exit()


if __name__=='__main__':
    if not os.path.exists(cfg.imgSaveDir):
        os.makedirs(cfg.imgSaveDir)
    if not os.path.exists(cfg.audioDir):
        os.makedirs(cfg.audioDir)
    if not os.path.exists(cfg.vidSaveDir):
        os.makedirs(cfg.vidSaveDir)

    sen = Sentry()
    if cfg.TEST_MODE:
        testingMode(sen)
    # TODO: Insert code to check for bluetooth controller to enter wrangler mode
    sen.mainLoop()