import Config as cfg
import playsound
import simpleaudio as sa

class AudioPlayer(object):
    def __init__(self):
        self.spotSound = sa.WaveObject.from_wave_file(cfg.audioDir + "Sentry_spot.wav")
        self.scanSound = sa.WaveObject.from_wave_file(cfg.audioDir + "Sentry_scan.wav")

    def playScanSound(self):
        play_obj = self.spotSound.play()
        # playsound(cfg.audioDir + self.scanSound)

    def playSpotSound(self):
        play_obj = self.scanSound.play()
        # playsound(cfg.audioDir + self.spotSound)