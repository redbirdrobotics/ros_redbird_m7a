#!/usr/bin/python
import rospy
import numpy as np
import cv2


class Camera(object):
    #ident: Unique identity
    #hRes, vRes: HORIZONTAL AND VERTICAL RESOLUTION
    #hRange, vRange: HORIZONTAL AND VERTICAL RANGE OF LENS IN DEGREES
    #azimuth: AZIMUTHAL ANGLE IN DEGREES, 0 IS FACING FORWARD RELATIVE TO DRONE ORIENTATION PROGRESSING CLOCKWISE
    #altitude: ALTITUDINAL ANGLE IN DEGREE, 0 IS FACING DOWNWARD RELATIVE TO DRONE ORIENTATION PROGRESSING UP
    def __init__(self, ident, (hRes, vRes), (hRange, vRange), (azimuth, altitude)):
        self.ident = ident
        self.lensRange = np.radians(hRange), np.radians(vRange)
        self.orientation = np.radians(azimuth), np.radians(altitude)
        self.xAxis = np.zeros((1, hRes))
        self.yAxis = np.zeros((1, vRes))
        self.hRes = hRes
        self.vRes = vRes

        self.createAxis()
        return

    # def detach(self):
    #     self.feed.release()
    #     return

    # def getFrame(self):
    #     ret, frame = self.feed.read()
    #     return frame

    # @staticmethod
    # def getFrameList(camList, frameList):
    #     Utilities.emptyList(frameList)
    #     length = len(camList)
    #     for c in range(length):
    #         ret, frame = camList[c].feed.read()
    #         frameList.append(frame)
    #     return

    @staticmethod
    def showFrame(frame, str):
        cv2.imshow(str, frame)
        k = cv2.waitKey(30) & 0xff
        esc = False
        if k == 27:
            esc = True
        return esc

    # def getRes(self):
        # hRes = int(self.feed.get(cv2.CAP_PROP_FRAME_WIDTH))
        # vRes = int(self.feed.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # return hRes, vRes

#_______________________________________________________#
    #Pixel Mapping
#---------------------------------------------------#

    def createAxis(self):
        #hRes, vRes = self.getRes()

        #ROS
        hRes = self.hRes
        vRes = self.vRes

        hRange, vRange = self.lensRange
        hResMid = int(hRes/2)
        vResMid = int(vRes/2)
        hRangeMid = int(hRange/2)
        vRangeMid = int(vRange/2)
        az, al = self.orientation

        #Create X & Y Axis of Angle Map

        #X AXIS
        self.xAxis = np.linspace((-hRangeMid + az), (hRangeMid + az), hRes)

        #Y AXIS
        self.yAxis = np.linspace((vRangeMid + al), (-vRangeMid + al), vRes)
        return
