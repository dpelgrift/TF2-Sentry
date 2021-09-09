__version__ = '0.0.1'

import Config as cfg
from simple_pid import PID
import os
import time
import math


class MotorDriver(object):
    def __init__(self, serialDevice):
        self.serialDevice = serialDevice
        self.yawTarget = 0.
        self.pitchTarget = 0.
        self.yawVel = 0.
        self.tiltVel = 0.
        self.spool = 0
        self.triggerStart = 0
        self.isParked = False



    def configItsybitsy(self):
        print()


    def zero(self):
        print()

    