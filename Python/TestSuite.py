import Config as cfg
import time

testModeCommandString = "1: Test audio, 2: Test hopper, 3: Test pusher\n"+\
                        "4: Test flywheels, 5: Test serial connection, 6: Test IMU\n"+\
                        "7: Test tilt servo, 8: Test pan stepper, 9: Test firing sequence"
 

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
        testSerial(sentry)
    elif val == 6:
        testIMU(sentry)
    elif val == 7:
        testTilt(sentry)
    elif val == 8:
        testPan(sentry)
    elif val == 9:
        testFiring(sentry)


def testAudio(sentry):
    print("Playing scan sound:")
    sentry.audio.playScanSound()
    time.sleep(1.0)
    print("Playing spot sound:")
    sentry.audio.playSpotSound()

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

def testSerial(sentry):
    print()

def testIMU(sentry):
    print()

def testTilt(sentry):
    print()

def testPan(sentry):
    print()

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