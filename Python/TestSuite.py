import Config as cfg
import AudioPlayer
import time

testModeCommandString = "1: Test audio, 2: Test Hopper\n 3: Test pusher, 4: Test flywheels\n5: Test serial connection, 6: Test IMU\n7: Test tilt servo, 8: Test pan stepper"
 

def testingMode(motors):
    print("Test Mode Entered")
    while True:
        print(testModeCommandString)
        val = input("Enter value 1-8 to enter specific testing mode: ")
        testSelector(int(val),motors)
    

def testSelector(val,motors):
    if val==1:
        testAudio()
    elif val==2:
        print()
    elif val==3:
        print()
    elif val==4:
        print()
    elif val == 5:
        print()
    elif val == 6:
        print()
    elif val == 7:
        print()
    elif val == 8:
        print()


def testAudio():
    ap = AudioPlayer()
    print("Playing scan sound:")
    ap.playScanSound()
    time.sleep(1.0)
    print("Playing spot sound:")
    ap.playSpotSound()

def testHopper(motors):
    val = input("Select input to hopper servo (-1 to 1): ")
    motors.hopper.value = int(val)
    time.sleep(5.0)
    motors.hopper.value = int(0)

def testPusher(motors):
    val = input("Select input to hopper servo (-1 to 1): ")
    motors.hopper.value = int(val)
    time.sleep(5.0)
    motors.hopper.value = int(0)