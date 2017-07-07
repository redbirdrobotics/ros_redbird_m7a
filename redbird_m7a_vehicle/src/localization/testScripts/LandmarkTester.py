import cv2
import numpy as np

class Landmark():

    def __init__(self):
        self.goalVals = (1, 180, 100)
        self.gridVals = (0,0,0)
        self.endPoints = (0,0,0,0)
        self.locDataList = []
        self.found = False
        return

    def detectGoalLine(self, imgList):
        rhoArg, thetaArg, minLenArg = self.goalVals

        for img in imgList:
            if type(img) == tuple:
                cam = img[0]
                color = img[1]

            else:
                lines = cv2.HoughLines(img, rhoArg, np.pi/thetaArg, minLenArg)            
                if not (lines == None):
                    self.found = True
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
                        self.cam = cam
                        self.color = color
                        self.endpoints = (xA, yA, xB, yB)
                        self.locDataList.append(cam, color)
                        self.locDataList.append(endpoints)
                else:
                    self.found = False
                    self.endPoints = (0,0,0,0)
        return

    def drawLine(self, img, w):
        xA, yA, xB, yB = self.endPoints
        cv2.line(img, (xA, yA), (xB, yB), (0,0,0), w)
        return

    def removeFromImgList(self, imgList):
        for img in imgList:
            
            img = self.drawLine(img, 15)
        return







        
        
