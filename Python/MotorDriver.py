import Config as cfg
import time
from numpy import pi
from gpiozero import Servo, LED


class MotorDriver(object):
    def __init__(self, serial_device):

        self.sd = serial_device

        self.pusher = Servo(cfg.pusherPin,min_pulse_width=cfg.pusherMinPwm,max_pulse_width=cfg.pusherMaxPwm)
        self.hopper = Servo(cfg.hopperPin,min_pulse_width=cfg.hopperMinPwm,max_pulse_width=cfg.hopperMaxPwm)

        self.hopper.value = None
        self.pusher.value = None
        
        self.flyWheels = LED(cfg.flywheelPin)
        self.pusherDirection = 1

        self.led = LED(cfg.ledPin)
        self.led.on()
        
        self.laser = LED(cfg.laserPin)
        if cfg.doTurnLaserOn:
            self.laser.on()

    def zero(self):
        self.sd.command('G0 X0 Y0')

    def move(self, targetYaw, targetTilt):
        self.sd.command(f'G0 X{targetYaw} Y{targetTilt}')

    def setYaw(self,yawDeg):
        self.sd.command(f'G1 X{yawDeg:.2f}')

    def startScan(self):
        self.sd.command('G2')

    def haltInPlace(self):
        self.sd.command('G3')

    def initMPU(self):
        self.sd.command('M0',False)
        ret = self.sd.waitForResponse('mpu ok',5.0)
        return ret

    def resetDMP(self):
        self.sd.command('M1')

    def configTiltServo(self):
        self.sd.command('M2',False)
        ret = self.sd.waitForResponse('tilt ok',15.0)
        return ret

    def setSpeedParams(self,steps_per_rev=None,yaw_speed=None,yaw_accel=None,tilt_speed=None):
        cmd = 'M3'
        if not steps_per_rev == None:
            cmd += f' A{steps_per_rev}'
        if not yaw_speed == None:
            cmd += f' B{yaw_speed}'
        if not yaw_accel == None:
            cmd += f' C{yaw_accel}'
        if not tilt_speed == None:
            cmd += f' D{tilt_speed}'

        self.sd.command(cmd)

    def getCurrYawPitch(self):
        resp = self.sd.command('M4')
        vals = resp.split(',')
        return float(vals[0]), float(vals[1])

    def getCurrYawVel(self):
        resp = self.sd.command('M5')
        return float(resp)

    def startFiring(self):
        self.hopper.value = 1
        self.pusher.value = self.pusherDirection

    def stopFiring(self):
        self.hopper.value = None
        self.pusher.value = None

    def reversePusher(self):
        self.pusherDirection = -1*self.pusherDirection
        self.pusher.value = self.pusherDirection

    def blinkErrCode(self,errCode):
        # Err types & codes:
        # Serial connection err:    0
        # Itsybitsy config err:     1
        # Camera init err:          2
        #

        while True:
            self.led.off()
            time.sleep(1)

            for i in range(errCode+1):
                self.led.on()
                time.sleep(0.2)
                self.led.off()
                time.sleep(0.2)


    

    