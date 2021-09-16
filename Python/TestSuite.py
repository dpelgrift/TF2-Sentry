import Config as cfg
from AudioPlayer import *
import time

testModeCommandString = "1: Test audio, 2: Test Hopper, 3: Test pusher\n 4: Test flywheels, 5: Test serial connection, 6: Test IMU\n7: Test tilt servo, 8: Test pan stepper, 9: Test firing sequence"
 

def testingMode(motors,sd):
    print("Test Mode Entered")
    while True:
        print(testModeCommandString)
        val = input("Enter value 1-9 to enter specific testing mode: ")
        testSelector(int(val),motors,sd)
    

def testSelector(val,motors,sd):
    if   val == 1:
        testAudio()
    elif val == 2:
        testHopper(motors)
    elif val == 3:
        testPusher(motors)
    elif val == 4:
        testFlywheels(motors)
    elif val == 5:
        testSerial(sd)
    elif val == 6:
        testIMU(sd)
    elif val == 7:
        testTilt(sd)
    elif val == 8:
        testPan(sd)
    elif val == 9:
        testFiring(motors)


def testAudio():
    ap = AudioPlayer()
    print("Playing scan sound:")
    ap.playScanSound()
    time.sleep(1.0)
    print("Playing spot sound:")
    ap.playSpotSound()

def testHopper(motors):
    val = input("Specify input to hopper CR servo (-1 to 1): ")
    motors.hopper.value = int(val)
    time.sleep(5.0)
    motors.hopper.value = 0.

def testPusher(motors):
    val = input("Specify input to pusher CR servo (-1 to 1): ")
    motors.pusher.value = int(val)
    time.sleep(5.0)
    motors.pusher.value = 0.

def testFlywheels(motors):
    print("Testing flywheel motors for 5 seconds")
    motors.flyWheels.on()
    time.sleep(5.0)
    motors.flyWheels.off()

def testSerial(sd):
    print()

def testIMU(sd):
    print()

def testTilt(sd):
    print()

def testPan(sd):
    print()

def testFiring(motors):
    hopperVal = input("Specify input to hopper CR servo (-1 to 1): ")
    pusherVal = input("Specify input to pusher CR servo (-1 to 1): ")
    spoolTime = input("Specify spool-up period for flywheels (sec): ")
    fireTime = input("Specify firing period (sec): ")

    motors.flyWheels.on()
    time.sleep(float(spoolTime))

    motors.hopper.value = int(hopperVal)
    motors.pusher.value = int(pusherVal)
    time.sleep(float(fireTime))
    motors.flyWheels.off()
    motors.hopper.value = 0.
    motors.pusher.value = 0.