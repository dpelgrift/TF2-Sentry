__version__ = '0.0.1'

import time
import os
import cv2
from PIL import Image
import Config as cfg
import numpy as np


class CameraDriver(object):
    def __init__(self, res=cfg.videoResolution):
        self.resolution = res
        self.isEnabled = False
        self.capture = None

        self.tlast = time.time()
        self.frameNum = 0
        self.targetLocked = False

        self.fullBodyCascade = cv2.CascadeClassifier('haarcascade_fullbody.xml')
        self.tracker = cv2.TrackerKCF_create()
        
        #self.upperBodyCascade = cv2.CascadeClassifier('haarcascade_upperbody.xml')

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

    def resetLock(self):
        self.tracker.clear()
        self.tracker = cv2.TrackerKCF_create()
        self.lockedOn = False

    def lockOn(self,targetBox,frame):
        self.resetLock()
        self.tracker.init(frame,targetBox)
        self.lockedOn = True

    def getTargetLocation(self):
        frame = self.getFrame()
        ok, bbox = self.tracker.update(frame)

        tnow = time.time()
        if ok:
            h = bbox[1] + int(bbox[3] / 2)
            w = bbox[0] + int(bbox[2] / 2)

            (a, b, c, d) = (int(j) for j in bbox)
            
            if cfg.DEBUG_MODE:
                print("[{}, {}] - {} fps".format(h, w, 1 / (tnow - self.tlast)))
                self.tlast = tnow
                
            if cfg.SAVE_IMGS:
                frame = cv2.rectangle(frame, (a, b), (a + c, b + d), (0, 0, 0), 2)
                self.saveImage(frame, 'cv_{}.jpg'.format(self.frameNum))
            return (h, w)
        else:
            print("Tracking failed")
            if cfg.SAVE_IMGS:
                self.saveImage(frame, 'cv_{}.jpg'.format(self.frameNum))
            return (0,0)

    def findTarget(self):
        frame = self.getFrame()

        fullBodyTargets = self.fullBodyCascade.detectMultiScale(frame,1.2,6)
        # upperBodyTargets = self.upperBodyCascade.detectMultiScale(frame,1.2,6)

        # If targets empty, return None
        if len(fullBodyTargets) == 0:
            return None, frame
        if len(fullBodyTargets) == 1: # If only one target detected, return it
            [a, b, c, d] = fullBodyTargets[0]
            return (a, b, c, d), frame

        # Otherwise, find the target closest to center of frame
        midPoint = np.around(np.array(cfg.videoResolution)/2)
        targetArray = np.array(fullBodyTargets)
        
        numTargets = targetArray.shape[0]
        targetDists = np.zeros(numTargets)
        targetLocations = targetArray[:,0:2] + np.around(targetArray[:,2:4]/2) - midPoint
        
        # Compute vector magnitude of distance to each target
        for tIdx in range(numTargets):
            targetDists[tIdx] = np.linalg.norm(targetLocations[tIdx,:1])
            
        [a, b, c, d] = fullBodyTargets[np.argmin(targetDists),:]
        return (a, b, c, d), frame



    def getFrame(self):
        _, frame = self.capture.read()
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if cfg.SAVE_IMGS:
            self.frameNum += 1
            self.saveImage(frame, '{}.jpg'.format(self.frameNum))

        return grayFrame