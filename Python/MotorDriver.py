__version__ = '0.0.1'

import Config as cfg
from gpiozero import Servo, LED


class MotorDriver(object):
    def __init__(self):

        self.pusher = Servo(cfg.pusherPin,min_pulse_width=cfg.pusherMinPwm,max_pulse_width=cfg.pusherMaxPwm)
        self.hopper = Servo(cfg.hopperPin,min_pulse_width=cfg.hopperMinPwm,max_pulse_width=cfg.hopperMaxPwm)
        
        self.flyWheels = LED(cfg.flywheelPin)
        self.pusherDirection = 1

        self.led = LED(cfg.ledPin)
        self.led.on()
        
        self.laser = LED(cfg.laserPin)
        if cfg.doTurnLaserOn:
            self.laser.on()


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

        #TODO
        print()


    

    