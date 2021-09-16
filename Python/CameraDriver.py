__version__ = '0.0.1'

import sys
import time
import os
import cv2
from PIL import Image
import atexit
import Config as cfg
import numpy as np
import subprocess as sp


class CameraDriver(object):
    def __init__(self, res=cfg.videoResolution):
        self.resolution = res
        self.isEnabled = False

        self.frameNum = 0

        self.targetLocked = False

        #self.tracker = cv2.TrackerKCF_create()
        self.fullBodyCascade = cv2.CascadeClassifier('haarcascade_fullbody.xml')
        self.upperBodyCascade = cv2.CascadeClassifier('haarcascade_upperbody.xml')

    @staticmethod
    def saveImage(img, imPath):
        im = Image.fromarray(img)
        im.save(os.path.join(cfg.imgSaveDir, imPath))
    
    
    def start(self):
        self.capture = cv2.VideoCapture(0)

        w, h = self.resolution
        self.capture.set(3,w)
        self.capture.set(4,h) 

    def stop(self):
        self.capture.release()


    def findTargets(self):
        frame = self.getFrame()

        fullBodyTargets = self.fullBodyCascade.detectMultiScale(frame,1.2,6)
        # upperBodyTargets = self.upperBodyCascade.detectMultiScale(frame,1.2,6)



    def getFrame(self):
        _, frame = self.capture.read()
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        return grayFrame