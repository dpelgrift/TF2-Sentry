__version__ = '0.0.1'

import Config as cfg
import os
import time
import math
from gpiozero import Servo, LED


class MotorDriver(object):
    def __init__(self, serialDevice):
        self.serialDevice = serialDevice
        self.yawTarget = 0.
        self.pitchTarget = 0.
        self.yawVel = 0.
        self.tiltVel = 0.
        self.doSpool = 0
        self.triggerStart = 0

        self.pusher = Servo(cfg.pusherPin,min_pulse_width=cfg.pusherMinPwm,max_pulse_width=cfg.pusherMaxPwm)
        self.hopper = Servo(cfg.hopperPin,min_pulse_width=cfg.hopperMinPwm,max_pulse_width=cfg.hopperMaxPwm)
        
        self.flyWheels = LED(cfg.flywheelPin)

        self.led = LED(cfg.ledPin)
        self.led.on()
        self.laser = LED(cfg.laserPin)
        self.laser.on()


    def startFiring(self):
        print()

    def stopFiring(self):
        print()

    def reversPusher(self):
        print()

    def configItsybitsy(self):
        print()

    def zero(self):
        print()

    

    