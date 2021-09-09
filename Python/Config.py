""" MECHANICAL PARAMS """
panStepsPerRev = 400
panTravelWidthDeg = 180

panDefaultVelDegPerSec = 60
panDefaultVelDegPerSec2 = 200

flywheelMinSpoolTimeSec = 2

tiltMinPwm = 50.0
tiltMaxPwm = 100.0
tiltDefaultCenterPwm = 75

tiltDefaultVelPwmPerSec = 5
tiltDefaultAccPwmPerSec2 = 10

pusherReverseDurationSec = 3
pusherPwm = 100

hopperLeadingBufferSec = 1
hopperLaggingBufferSec = 1
hopperPwm = 100

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
audioPath = '/home/pi/audio'
imgSavePath = '/home/pi/imgs'
vidSavePath = '/home/pi/vids'

""" DEBUG """
DEBUG_MODE = True
SAVE_IMGS = True
SAVE_VID = True