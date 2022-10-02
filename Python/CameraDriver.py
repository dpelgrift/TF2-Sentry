from picamera import PiCamera
from picamera.array import PiRGBArray
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
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)
        self.camera.resolution = cfg.videoResolution
        
        self.tlast = time.time()
        self.frameNum = 0
        self.targetLocked = False

        self.fullBodyCascade = cv2.CascadeClassifier(cfg.cascadeModelPath)
        # self.tracker = cv2.TrackerKCF_create()
        if cfg.DISP_FRAME:
            self.rawFrameWin = cv2.namedWindow('rawframes',flags=cv2.WINDOW_AUTOSIZE)
            self.targetFrameWin = cv2.namedWindow('targetframes',flags=cv2.WINDOW_AUTOSIZE)
        

    @staticmethod
    def saveImage(img, imPath):
        im = Image.fromarray(img)
        im.save(os.path.join(cfg.imgSaveDir, imPath))
    
    
    def start(self):
        pass

        # config = self.camera.create_preview_configuration(main={"size": cfg.videoResolution, "format": "BGR888"})
        # self.camera.configure(config)
        # self.camera.start()

        # self.capture = cv2.VideoCapture(0)

        # w, h = self.resolution
        # self.capture.set(3,w)
        # self.capture.set(4,h) 

    def stop(self):
        self.capture.release()

    def resetLock(self):
        self.tracker.clear()
        self.tracker = cv2.TrackerKCF_create()
        self.targetLocked = False

    def lockOn(self,targetBox,frame):
        self.resetLock()
        self.tracker.init(frame,targetBox)
        self.targetLocked = True

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
            if cfg.DISP_FRAME:
                self.dispTargetFrame(frame,[a,b,c,d])
            return (h, w)
        else:
            print("Tracking failed")
            if cfg.SAVE_IMGS:
                self.saveImage(frame, 'cv_{}.jpg'.format(self.frameNum))
            return (0,0)

    def findTarget(self):

        frame = self.getFrame()

        print(f'In findTarget: frame.shape = {frame.shape}')

        fullBodyTargets = self.fullBodyCascade.detectMultiScale(frame,cfg.scaleFactor,cfg.minNeighbors)

        # If targets empty, return None
        if len(fullBodyTargets) == 0:
            # if cfg.DEBUG_MODE:
            #     print("No targets")

            return None, frame

        if len(fullBodyTargets) == 1: # If only one target detected, return it
            # if cfg.DEBUG_MODE:
            #     print("Target found")
            [a, b, c, d] = fullBodyTargets[0]

        else:
            # Otherwise, get the target closest to center of frame
            # midPoint = np.around(np.array(cfg.videoResolution)/2)
            midPoint = np.array([cfg.wTargetCenter, cfg.hTargetCenter])
            targetArray = np.array(fullBodyTargets)
            
            numTargets = targetArray.shape[0]
            targetDists = np.zeros(numTargets)
            targetLocations = targetArray[:,0:2] + np.around(targetArray[:,2:4]/2) - midPoint
            
            # Compute vector magnitude of distance to each target
            for tIdx in range(numTargets):
                targetDists[tIdx] = np.linalg.norm(targetLocations[tIdx,:1])
                
            [a, b, c, d] = fullBodyTargets[np.argmin(targetDists),:]

        if cfg.SAVE_IMGS:
            frameTmp = cv2.rectangle(frame, (a, b), (a + c, b + d), (0, 0, 0), 2)
            self.saveImage(frameTmp, 'cv_detect_{}_T{:.2f}.jpg'.format(self.frameNum,time.time()-cfg.t0))

        return (a, b, c, d), frame

    def dispTargetFrame(self,frame,bbox):
        a = bbox[0]
        b = bbox[1]
        c = bbox[2]
        d = bbox[3]
        rectFrame = cv2.rectangle(frame, (a, b), (a + c, b + d), (0, 0, 0), 2)
        rectFrame = cv2.circle(frame,(cfg.wTargetCenter, cfg.hTargetCenter),3,(0.1,0.1,0.1),thickness=3)
        cv2.imshow('targetframes',rectFrame)
        cv2.waitKey(1)

    def getFrame(self):

        t1 = time.time()
        self.camera.capture(self.rawCapture, format="bgr")
        frame = self.rawCapture.array
        # frame = self.camera.capture_array()
        # _,frame = self.capture.read()
        t2 = time.time()

        if cfg.DEBUG_MODE:
            print(f'Camera read time: {t2-t1}')

        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        self.rawCapture.truncate(0)

        if cfg.DISP_FRAME:
            cv2.imshow('rawframes',frame)
            cv2.waitKey(1)
            # cv2.imshow(self.rawFrameWin,frame)

        if cfg.SAVE_IMGS:
            self.frameNum += 1
            self.saveImage(frame, '{}.jpg'.format(self.frameNum))

        return grayFrame