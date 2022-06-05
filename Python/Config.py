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
videoResolution = (768,432)
hTargetCenter = videoResolution[1]/2
wTargetCenter = videoResolution[0]/2

horizFov = 124
vertFov = 93

# cascadeModelPath = 'haarcascade_fullbody.xml' 
cascadeModelPath = 'haarcascade_frontalface_default.xml' 
onTargetPixelProximity = 20
scaleFactor = 1.2
minNeighbors = 5

""" MISC """
doTurnLaserOn = True
spoolDownDelay = 5
scanSoundPlayInterval = 5
updateRateSec = 0.2

""" PID PARAMS """
DISABLE_PID = True
kp = 1500/1000000
ki = 350/1000000
kd = 5/1000000

""" PATHS """
audioDir = '/home/pi/audio/'
imgSaveDir = '/home/pi/imgs/'
vidSaveDir = '/home/pi/vids/'

""" SERIAL PARAMS """
BAUDRATE = 115200
S_TIMEOUT = 0.2
R_TIMEOUT = 10

""" DEBUG """
t0 = 0
DEBUG_MODE = True
TEST_MODE = False
SAVE_IMGS = True
SAVE_VID = True
DISP_FRAME = False
BYPASS_SERIAL_VERIFY = False