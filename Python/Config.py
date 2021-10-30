""" MECHANICAL PARAMS """
scanSpeedDegPerSec = 50
scanAngleDeg = 150

yawStepsPerRev = 400
yawTravelWidthDeg = 180

pitchLimitsDeg = (-10,20)

yawDefaultVelDegPerSec = 60
yawDefaultVelDegPerSec2 = 200

flywheelMinSpoolTimeSec = 2

tiltMinPwm = 0.5/1000
tiltMaxPwm = 2.5/1000
tiltDefaultCenterPwm = 75

tiltDefaultVelPwmPerSec = 5
tiltDefaultAccPwmPerSec2 = 10

pusherReverseDurationSec = 3
pusherMinPwm = 1.0/1000
pusherMaxPwm = 2.0/1000

hopperLeadingBufferSec = 1
hopperLaggingBufferSec = 1
hopperMinPwm = 1.0/1000
hopperMaxPwm = 2.0/1000

""" PINOUTS """
ledPin = 17
laserPin = 4
flywheelPin = 5
pusherPin = 6
hopperPin = 13

""" CAMERA """
videoResolution = (640,480)
hTargetCenter = 240
wTargetCenter = 320

horizFov = 124
vertFov = 93

onTargetPixelProximity = 20

""" MISC """
scanReturnPeriodSec = 3
scanSoundPlayInterval = 5

""" PID PARAMS """
kp = 1500/1000000
ki = 350/1000000
kd = 5/1000000

""" PATHS """
audioDir = '/home/pi/audio/'
imgSaveDir = '/home/pi/imgs/'
vidSaveDir = '/home/pi/vids/'

""" DEBUG """
DEBUG_MODE = True
TEST_MODE = True
SAVE_IMGS = True
SAVE_VID = True