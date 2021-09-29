__version__ = '0.0.1'

import serial
import serial.tools.list_ports
import Config as cfg
import time


BAUD = 250000
S_TIMEOUT = 0.2   # Serial command timeout
R_TIMEOUT = 10  # Wait for reponse timeout

class SerialDevice(object):
    def __init__(self):
        # self.comport = self.get_com_port()
        self.serial_dev = serial.Serial('/dev/serial0', BAUD, timeout=S_TIMEOUT)
        self.R_TIMEOUT= R_TIMEOUT

    def command(self, data_string):
        self.serial_dev.flush()
        self.serial_dev.write((data_string + '\n').encode('ascii'))
        tnow = time.time()
        last_resp = ''
        while time.time() - tnow < R_TIMEOUT:
            if (self.serial_dev.inWaiting() > 0):  
                resp = self.serial_dev.readline().decode('ascii')
                if 'ok' in resp:
                    return last_resp
                else:
                    last_resp = resp
                    if cfg.DEBUG_MODE:
                        print('serial response:', last_resp)

    def verifySerial(self):
        self.serial_dev.flush()
        self.serial_dev.write(('marco\n').encode('ascii'))
        tnow = time.time()
        while time.time() - tnow < self.R_TIMEOUT:
            if (self.serial_dev.inWaiting() > 0):  
                resp = self.serial_dev.readline().decode('ascii')
                if 'polo' in resp:
                    return True
                else:
                    print("Serial Response:" + resp)
        return False