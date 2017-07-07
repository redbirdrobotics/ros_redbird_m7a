import cv2
import numpy as np

class Landmark():

    def __init__(self):
        self.cam = None
        self.goalVals = (1, 180, 100)
        self.gridVals = (0,0,0)
        self.endPoints = (0,0,0,0)
        self.found = False
        return

    def detectGoalLine(self, imgList):
        rhoArg, thetaArg, minLenArg = self.goalVals
        for img in imgList:
            if type(img) == tuple:
                cam, mask = img

            else:
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
                else:
                    self.found = False
                    self.endPoints = (0,0,0,0)
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







        
        
