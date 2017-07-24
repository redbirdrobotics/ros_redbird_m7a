import cv2
import numpy as np
from Utilities import Utilities

class Landmark(object):
    def __init__(self, color, valArray):
        self.color = color
        self.minThresh = valArray[0]
        self.maxThresh = valArray[1]
        self.cam = None
        self.lineVals = (1, 180, 100)
        self.endPoints = (0,0,0,0)
        self.mendPoints = (0,0,0,0)
        self.found = False
        return

    def createHSVMask(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.minThresh, self.maxThresh)
        return mask

    def getMaskList(self, frameList, maskList):
        Utilities.emptyList(maskList)
        for frame in frameList:
            maskList.append( self.createHSVMask(frame) )
        return

    def detectGoalLine(self, imgList, camList):
        rhoArg, thetaArg, minLenArg = self.lineVals
        cam = 0
        for img in imgList:

            lines = cv2.HoughLines(img, rhoArg, np.pi/thetaArg, minLenArg)
            
            if not (lines == None):
                self.found = True
                self.cam = cam
                for rho, theta in lines[0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    xN = a*rho
                    yN = b*rho
                  
                    if xN < 0:
                        xN = -xN
                  
                    if yN < 0:
                        yN = -yN
                    
                    hRes = camList[cam].hRes
                    vRes = camList[cam].vRes

                    xA = int(xN + 2000*(-b))
                    yA = int(yN + 2000*(a))
                    xB = int(xN - 2000*(-b))
                    yB = int(yN - 2000*(a))

                    self.scale2Frame(xA, yA, xB, yB, camList)
                    cam += 1
            else:
                self.found = False
                self.endPoints = (0,0,0,0)
                cam += 1
        return

    def scale2Frame(self, xA, yA, xB, yB, camList):
        hRes = camList[self.cam].hRes
        vRes = camList[self.cam].vRes
        slope = float(yB - yA)/(xB - xA)
        yInt = int(yA - slope*xA)
        print "yint", yInt
        print "slope", slope

        if yInt < 0:
            print 'o0'
            yA = 0
            xA = int(-yInt/slope)

            yB = 0
            xB = int(-yInt/slope) 

        if yInt >= 0:

            if xA < 0:
                print 'o1'
                xA = 0
                yA = yInt
                if yB > vRes:
                    yB = vRes - 1
                    xB = int((yB - yInt)/slope)

            elif yA < 0:
                print 'o2'
                yA = 0
                xA = int(-yInt/slope)

            elif yA > vRes:
                print 'o3'
                yA = vRes - 1
                xA = int((yB - yInt)/slope)

            if xB > hRes:
                print "o4"
                xB = hRes -1
                yB = int(slope * xB + yInt)
                if yB > vRes:
                    yB = vRes - 1
                    xB = int((yB - yInt)/slope)

            elif yB < 0:
                yB = 0
                xB = int(-yInt/slope)

            elif yB > vRes:
                yB = vRes -1
                xB = int((yB - yInt)/slope)

        self.endPoints = (xA, yA, xB, yB)
        return

    def drawLine(self, img, w):
        xA, yA, xB, yB = self.endPoints
        img = cv2.line(img, (xA, yA), (xB, yB), (0,0,0), w)
        return

    
    def remove(self, imgList, w):
        xA, yA, xB, yB = self.endPoints
        cam = self.cam
        imgList[self.cam] = cv2.line(imgList[self.cam], (xA, yA), (xB, yB), (0,0,0), w)
        return

    def cvt2meters(self, quadDataList, camList):

        if self.found == False:
            return

        #print quadDataList
        Qx, Qy, Qh, Qyaw, Qpitch, Qroll = quadDataList
        xAxis = camList[self.cam].xAxis
        xAxisQ = [x + Qyaw for x in xAxis]
        yAxis = camList[self.cam].yAxis
        yAxisQ = [y + Qpitch for y in yAxis]
        xA, yA, xB, yB = self.endPoints

        print "endPoints: ", self.endPoints
        print "len: xAxis", len(xAxis)
        print "len: yAxis", len(yAxis)

        thetaA = xAxisQ[xA]
        phiA = yAxisQ[yA]
        yDistA = Qh*np.tan(phiA)
        xDistA = Qh*np.tan(thetaA)

        thetaB = xAxisQ[xB]
        phiB = yAxisQ[yB]
        yDistB = quadDataList[2]*np.tan(phiB)
        xDistB = quadDataList[2]*np.tan(thetaB)

        self.mendPoints = (xDistA + quadDataList[0], yDistA + quadDataList[1], xDistB + quadDataList[0], yDistB + quadDataList[1])
        return

    @staticmethod
    def emptyList(anyList):
        maxElem = len(anyList)
        del anyList[0:maxElem]
        return
