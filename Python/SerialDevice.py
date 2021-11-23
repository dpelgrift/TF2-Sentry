__version__ = '0.0.1'

import serial
import serial.tools.list_ports
import Config as cfg
import time


class SerialDevice(object):
    def __init__(self):
        # self.comport = self.get_com_port()
        self.serial_dev = serial.Serial('/dev/ttyAMA0', cfg.BAUDRATE, timeout=cfg.S_TIMEOUT)
        self.R_TIMEOUT= cfg.R_TIMEOUT

    def command(self, data_string):
        self.serial_dev.flush()
        self.serial_dev.write((data_string + '\n').encode('ascii'))
        tnow = time.time()
        last_resp = ''
        while time.time() - tnow < self.R_TIMEOUT:
            if (self.serial_dev.inWaiting() > 0):  
                resp = self.serial_dev.readline().decode('ascii')
                if 'ok' in resp:
                    return last_resp
                else:
                    last_resp = resp
                    if cfg.DEBUG_MODE:
                        print('serial response:', last_resp)

    def verifySerial(self,msg1,msg2):
        self.serial_dev.flush()
        self.serial_dev.write((msg1).encode('ascii'))
        tnow = time.time()
        while time.time() - tnow < self.R_TIMEOUT:
            if (self.serial_dev.inWaiting() > 0):  
                resp = self.serial_dev.readline().decode('ascii')
                if msg2 in resp:
                    return True
                else:
                    print("Serial Response:" + resp)
        return False

    def readSerialLine(self):
        resp = ''
        if (self.serial_dev.inWaiting() > 0):
            resp = self.serial_dev.readline().decode('ascii')
        return resp
