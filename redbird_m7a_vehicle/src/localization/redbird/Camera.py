import cv2
import numpy as np
import math
import os
import time


#np.set_printoptions(threshold = np.nan)

class Camera():
    
    #PORT: USB PORT CAMERA IS ATTACHED TO
    #hRes, vRes: HORIZONTAL AND VERTICAL RESOLUTION
    #FPS: FRAMES PER SECOND
    #hRange, vRange: HORIZONTAL AND VERTICAL RANGE OF LENS IN DEGREES
    #azimuth: AZIMUTHAL ANGLE IN DEGREES, 0 IS FACING FORWARD RELATIVE TO DRONE ORIENTATION PROGRESSING CLOCKWISE
    #altitude: ALTITUDINAL ANGLE IN DEGREE, 0 IS FACING DOWNWARD RELATIVE TO DRONE ORIENTATION PROGRESSING UP
    def __init__(self, port, (hRes, vRes), FPS, (hRange, vRange), (azimuth, altitude)):
        self.port = port
        self.feed = cv2.VideoCapture(port)
        ret, self.frame = self.feed.read()
        self.hRes = self.feed.set(cv2.CAP_PROP_FRAME_WIDTH, hRes)
        self.vRes = self.feed.set(cv2.CAP_PROP_FRAME_HEIGHT, vRes)
        self.setFPS = self.feed.set(cv2.CAP_PROP_FPS, FPS)
        self.lensRange = np.radians(hRange), np.radians(vRange)
        self.orientation = np.radians(azimuth), np.radians(altitude)
        self.xAxis = np.zeros((1, hRes))
        self.yAxis = np.zeros((1, vRes))
        return 

    def detach(self):
        self.feed.release()
        return

    def getFrame(self):
        ret, frame = self.feed.read()       
        return frame
    
    @staticmethod
    def getFrameList(camList):
        frameList = []
        length = len(camList)
        for c in range(length):
            ret, frame = camList[c].feed.read()
            frameList.append(frame)
        return frameList
    
    @staticmethod
    def showFrame(frame, str):
        cv2.imshow(str, frame)
        k = cv2.waitKey(30) & 0xff
        esc = False
        if k == 27:
            esc = True
        return esc
        
    def getRes(self):
        hRes = int(self.feed.get(cv2.CAP_PROP_FRAME_WIDTH))
        vRes = int(self.feed.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return hRes, vRes
    
#_______________________________________________________#
    #Pixel Mapping 
#---------------------------------------------------#

    def create_angleAxis(self):
        #GET VARIABLES
        hRes, vRes = self.getRes()
        hRange, vRange = self.lensRange
        hRes_Mid = hRes/2
        vRes_Mid = vRes/2
        hRange_Mid = hRange/2
        vRange_Mid = vRange/2
        az, al = self.orientation

        #Create X & Y Axis of Angle Map
        
        #X AXIS
        self.xAxis = np.linspace((-hRange_Mid + az), (hRange_Mid + az), hRes)
        
        #Y AXIS
        self.yAxis = np.linspace((vRange_Mid + al), (-vRange_Mid + al), vRes)
        #print "Cam yAxis from" ,(vRange_Mid + al) ,"to", (-vRange_Mid + al), "with" ,hRes ,"Values"
        return

