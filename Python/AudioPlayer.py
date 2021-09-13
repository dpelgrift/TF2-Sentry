import Config as cfg
import playsound


class AudioPlayer(object):
    def __init__(self):
        self.spotSound = 'Sentry_spot.wav'
        self.scanSound = 'Sentry_scan.wav'

    def playScanSound(self):
        playsound(cfg.audioDir + self.scanSound)

    def playSpotSound(self):
        playsound(cfg.audioDir + self.spotSound)