import Config as cfg
#import playsound
import pygame



def playScanSound():
    #play_obj = self.spotSound.play()
    pygame.mixer.init()
    pygame.mixer.music.load(cfg.audioDir + "Sentry_scan.wav")
    pygame.mixer.music.play()
    #while pygame.mixer.music.get_busy():
    #    continue

def playSpotSound():
    #play_obj = self.scanSound.play()
    pygame.mixer.init()
    pygame.mixer.music.load(cfg.audioDir + "Sentry_spot.wav")
    pygame.mixer.music.play()
    #while pygame.mixer.music.get_busy():
    #    continue

def playErrorSound():
    #play_obj = self.scanSound.play()
    pygame.mixer.init()
    pygame.mixer.music.load(cfg.audioDir + "Sentry_error.wav")
    pygame.mixer.music.play()
    #while pygame.mixer.music.get_busy():
    #    continue