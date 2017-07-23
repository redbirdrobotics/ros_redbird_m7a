import cv2
import numpy as np

class LandmarkLib():

    def __init__(self, valArray):
        self.minThresh = valArray[0]
        self.maxThresh = valArray[1]
        self.cam = None
        self.lineVals = (1, 180, 100)
        self.endPoints = (0,0,0,0)
        self.found = False
        return

    def createHSVMask(self, img):
        hsv = cv2.cvtcolor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.minThresh, self.maxThresh)
        return mask

    def getMaskList(self, frameList, maskList):
        Utilities.emptyList(maskList)
        for frame in frameList:
            maskList.append( self.createHSVMask(frame) )
        return

    def detectGoalLine(self, imgList):
        rhoArg, thetaArg, minLenArg = self.goalVals
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
                    
                    xA = int(xN + 2000*(-b))
                    yA = int(yN + 2000*(a))
                    xB = int(xN - 2000*(-b))
                    yB = int(yN - 2000*(a))
                    self.endPoints = (xA, yA, xB, yB)
                    cam += 1
            else:
                self.found = False
                self.endPoints = (0,0,0,0)
                cam += 1
        return

    def drawLine(self, img, w):
        xA, yA, xB, yB = self.endPoints
        img = cv2.line(img, (xA, yA), (xB, yB), (0,0,0), w)
        return img

    
    def remove(self, imgList, w):
        xA, yA, xB, yB = self.endPoints
        cam = self.cam
        addressList = [index for index, img in enumerate(imgList) if type(img) == tuple and img[0] == cam]
        for address in addressList:
            imgList[address + 1] = cv2.line(imgList[address + 1], (xA, yA), (xB, yB), (0,0,255), w)
        return imgList

    def cvt2meters(self, quadDataList, camList):
        Qx, Qy, Qh, Qyaw, Qpitch, Qroll
        xAxis = camList[self.cam].xAxis
        xAxisQ = [x + Qyaw for x in xAxis]
        yAxis = camList[self.cam].yAxis
        yAxisQ = [y + Qpitch for y in yAxis]
        xA, yA, xB, yB = self.endPoints

        thetaA = xAxisQ[xA]
        phiA = yAxisQ[yA]
        yDistA = quadDataList[2]*np.tan(phiA)
        xDistA = quadDataList[2]*np.tan(thetaA)

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
