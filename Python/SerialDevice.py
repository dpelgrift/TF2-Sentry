import serial
import serial.tools.list_ports
import Config as cfg
import time


class SerialDevice(object):
    def __init__(self):
        # self.comport = self.get_com_port()
        self.serial_dev = serial.Serial('/dev/ttyAMA0', cfg.BAUDRATE, timeout=cfg.S_TIMEOUT)
        self.R_TIMEOUT= cfg.R_TIMEOUT

    def command(self, data_string,doWaitForResp=True):
        self.serial_dev.flush()
        self.serial_dev.write((data_string + '\n').encode('ascii'))
        tnow = time.time()
        last_resp = ''
        if not doWaitForResp:
            return None
        while time.time() - tnow < self.R_TIMEOUT:
            if (self.serial_dev.inWaiting() > 0):  
                resp = self.serial_dev.readline().decode('ascii')
                if 'ok' in resp:
                    return last_resp
                else:
                    last_resp = resp
                    if cfg.DEBUG_MODE:
                        print('Serial Response:', last_resp)

    def verifySerial(self,msg1,msg2,timeOut=None):
        if timeOut == None:
            timeOut = self.R_TIMEOUT

        self.serial_dev.flush()
        self.serial_dev.write((msg1).encode('ascii'))

        return self.waitForResponse(msg2,timeOut)

    def waitForResponse(self,resp,timeOut=None):
        if timeOut == None:
            timeOut = self.R_TIMEOUT

        tnow = time.time()
        while time.time() - tnow < timeOut:
            if (self.serial_dev.inWaiting() > 0):  
                r = self.serial_dev.readline().decode('ascii')
                if resp in r:
                    return True
                else:
                    print('Serial Response:' + r)
        return False

    def readSerialLine(self):
        resp = ''
        if (self.serial_dev.inWaiting() > 0):
            resp = self.serial_dev.readline().decode('ascii')
        return resp
