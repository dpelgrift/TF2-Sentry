import Config as cfg
import os


class AudioPlayer(object):
    def __init__(self):
        self.spotSound = 'Sentry_spot.wav'
        self.scanSound = 'Sentry_scan.wav'

    def playScanSound(self):
        os.system("mpg123 " + cfg.audioDir + self.scanSound)

    def playSpotSound(self):
        os.system("mpg123 " + cfg.audioDir + self.spotSound)