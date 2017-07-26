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

        if xA == 0 and yA == 0 and xB == 0 and yB == 0:
            return

        if xA == xB or yA == yB:
            return

        hRes = camList[self.cam].hRes
        vRes = camList[self.cam].vRes
        slope = float(yB - yA)/(xB - xA)
        resSlope = float(vRes)/hRes
        yInt = int(yA - (slope * xA)) 
        xInt = int((-yInt)/(slope))
        #print 'pre endpoints',  xA, yA, xB, yB
        #print "yint", yInt
        #print "xInt", xInt
        #print "slope", slope


        if yInt < 0 :
            #print 'enters top'
            sxA = max(xInt, 0)
            syA = 0

            if (hRes * slope + yInt) < vRes:
                #print 'exits right'
                sxB = hRes -1
                syB = int(hRes * slope + yInt)

            elif (hRes * slope + yInt) > vRes:
                #print 'exits bottom'
                sxB = int((vRes - yInt)/slope)
                syB = vRes -1

            else:
                pass
                #print 'unaccounted'

        elif yInt >= 0 and yInt < vRes:
            #print 'enters left'
            sxA = 0
            syA = yInt

            if (hRes * slope + yInt) < 0:
                #print 'exits top'
                sxB = max(xInt, 0)
                syB = 0

            elif (hRes * slope + yInt) > 0 and (hRes * slope + yInt) < vRes:
                #print 'exits right'
                sxB = hRes -1
                syB = int(slope * hRes + yInt)

            elif (hRes * slope + yInt) > vRes:
                #print 'exits bottom'
                sxB = int((vRes - yInt)/slope)
                syB = vRes -1

            else:
                pass
                #print 'unaccounted'

        elif yInt > hRes:
            #print 'enters top'
            sxA = int((vRes - yInt)/slope)
            syA = vRes -1

            if (hRes * slope + yInt) > 0:
                #print 'exits right'
                sxB = hRes -1
                syB = int(hRes * slope + yInt)

            elif (hRes * slope + yInt) < 0:
                #print 'exits top'
                sxB = xInt -1
                syB = 0

            else: 
                pass
                #print 'unaccounted'

        else:
            self.endPoints = (0,0,0,0)
            return

        self.endPoints = (sxA, syA, sxB, syB)
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
        #print quadDataList
        xAxis = camList[self.cam].xAxis
        xAxisQ = [x + Qyaw for x in xAxis]
        yAxis = camList[self.cam].yAxis
        yAxisQ = [y + Qpitch for y in yAxis]
        xA, yA, xB, yB = self.endPoints

        # if xA < len(xAxis): xA = 0
        # if yA > len(yAxis): yA = len(yAxis)
        # if xB < len(xAxis): xB = 0
        # if yB > len(yAxis): yB = len(yAxis)

        #print "endPoints: ", self.endPoints

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
