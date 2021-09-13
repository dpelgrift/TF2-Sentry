""" MECHANICAL PARAMS """
panStepsPerRev = 400
panTravelWidthDeg = 180

panDefaultVelDegPerSec = 60
panDefaultVelDegPerSec2 = 200

flywheelMinSpoolTimeSec = 2

tiltMinPwm = 1.0/1000
tiltMaxPwm = 2.0/1000
tiltDefaultCenterPwm = 75

tiltDefaultVelPwmPerSec = 5
tiltDefaultAccPwmPerSec2 = 10

pusherReverseDurationSec = 3
pusherMinPwm = 0
pusherMaxPwm = 100

hopperLeadingBufferSec = 1
hopperLaggingBufferSec = 1
hopperMinPwm = 0
hopperMaxPwm = 100

""" PINOUTS """
ledPin = 17
laserPin = 4
flywheelPin = 5
pusherPin = 6
hopperPin = 13

""" CAMERA """
videoResolution = (640,480)
targetCenter = (320,240)

""" OPENCV PARAMS """
trackKp = 1500
trackKi = 350
trackKd = 5

allowedProximityPixels = 10
leadConstPixels = 15

""" PATHS """
audioDir = '/home/pi/audio'
imgSaveDir = '/home/pi/imgs'
vidSaveDir = '/home/pi/vids'

""" DEBUG """
DEBUG_MODE = True
TEST_MODE = False
SAVE_IMGS = True
SAVE_VID = True