import Config as cfg
import time
from AudioPlayer import *

testModeCommandString = "1: Test audio, 2: Test hopper, 3: Test pusher, 4: Test flywheels,\n"+\
                        "5: Test firing sequence, 6: Test camera, 7: Test target detection \n"+\
                        "8: Send move command, 9: Send multiple moves in a row, 10: Get current Attitude,\n" + \
                        "11: Send arbitrary gcode"

gcodeSpecString = "G0: Send move, G1: Set yaw, G2: Start Scan, G3: Stop scan\n" + \
                  "M0: Init MPU, M1: Reset DMP, M2: Config tilt servo,\n" + \
                  "M3: Set speed params, M4: Get current attitude, M5: Get current yaw velocity"

def testingMode(sentry):
    print("Test Mode Entered")
    while True:
        print(testModeCommandString)
        val = input("Enter value 1-9 to enter specific testing mode: ")
        testSelector(int(val),sentry)
    

def testSelector(val,sentry):
    if   val == 1:
        testAudio(sentry)
    elif val == 2:
        testHopper(sentry)
    elif val == 3:
        testPusher(sentry)
    elif val == 4:
        testFlywheels(sentry)
    elif val == 5:
        testFiring(sentry)
    elif val == 6:
        testCamera(sentry)
    elif val == 7:
        testTargetSearch(sentry)
    elif val == 8:
        sendMove(sentry)
    elif val == 9:
        testMultiCommand(sentry)
    elif val == 10:
        getCurrAttitude(sentry)
    elif val == 11:
        sendGCode(sentry)


def testAudio(sentry):
    print("Playing scan sound:")
    playScanSound()
    time.sleep(1.0)
    print("Playing spot sound:")
    playSpotSound()

def testHopper(sentry):
    val = input("Specify input to hopper CR servo (-1 to 1): ")
    sentry.motors.hopper.value = int(val)
    time.sleep(5.0)
    sentry.motors.hopper.value = 0.

def testPusher(sentry):
    val = input("Specify input to pusher CR servo (-1 to 1): ")
    sentry.motors.pusher.value = int(val)
    time.sleep(5.0)
    sentry.motors.pusher.value = 0.

def testFlywheels(sentry):
    print("Testing flywheel motors for 5 seconds")
    sentry.motors.flyWheels.on()
    time.sleep(5.0)
    sentry.flyWheels.off()


def sendMove(sentry):
    yaw = input("Specify absolute yaw (degrees): ")
    tilt = input("Specify absolute tilt (degrees): ")
    time.sleep(1.0)
    playSpotSound()
    time.sleep(1.0)
    sentry.move(float(tilt),float(yaw))
    # Pass through debug messages from ItsyBitsy
    initTime = time.time()
    while time.time() - initTime < 3:
        resp = sentry.sd.readSerialLine()
        if resp != '':
            print(resp)

    time.sleep(1.0)


def testMultiCommand(sentry):
    yaw = input("Specify absolute yaw (degrees): ")
    tilt = input("Specify absolute tilt (degrees): ")
    numCommands = input("Specify number of commands to send: ")
    time.sleep(1.0)
    playSpotSound()
    time.sleep(0.5)
    for c in range(int(numCommands)):
        sentry.move(float(tilt),float(yaw))
        time.sleep(0.5)
        resp = sentry.sd.readSerialLine()
        while resp != '':
            if resp != '':
                print(f'T: {time.time()-cfg.t0}, ' + resp)
            resp = sentry.sd.readSerialLine()

def getCurrAttitude(sentry):
    print("Getting attitude: ")
    yaw,pitch = sentry.motors.getCurrYawPitch()
    print(f'Yaw = {yaw}, Pitch = {pitch}')
    time.sleep(1.0)

def sendGCode(sentry):
    print(gcodeSpecString)
    comm = input("Specify command(s) to send: ")
    resp = sentry.motors.sd.command(comm)
    print(f'Resp = {resp}')
    # Pass through debug messages from ItsyBitsy
    initTime = time.time()
    while time.time() - initTime < 3:
        resp = sentry.sd.readSerialLine()
        if resp != '':
            print(resp)
    time.sleep(1.0)

def testCamera(sentry):
    sentry.cam.getFrame()
    time.sleep(1.0)

def testTargetSearch(sentry):
    time.sleep(1.0)
    print("Searching for targets:")
    initTime = time.time()
    while time.time() - initTime < 10:
        t1 = time.time()
        bbox, frame = sentry.cam.findTarget()
        t2 = time.time()
        print(f'Search took {t2-t1:.3f} sec')
        if bbox is not None:
            print(f'Target Found, bbox: {bbox}')

    time.sleep(1.0)

def testFiring(sentry):
    hopperVal = input("Specify input to hopper CR servo (-1 to 1): ")
    pusherVal = input("Specify input to pusher CR servo (-1 to 1): ")
    spoolTime = input("Specify spool-up period for flywheels (sec): ")
    fireTime = input("Specify firing period (sec): ")

    sentry.motors.flyWheels.on()
    time.sleep(float(spoolTime))

    sentry.motors.hopper.value = int(hopperVal)
    sentry.motors.pusher.value = int(pusherVal)
    time.sleep(float(fireTime))
    sentry.motors.flyWheels.off()
    sentry.motors.hopper.value = 0.
    sentry.motors.pusher.value = 0.