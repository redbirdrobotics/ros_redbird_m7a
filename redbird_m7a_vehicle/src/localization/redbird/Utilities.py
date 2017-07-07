import cv2
import math
import time
import numpy as np
from Camera import Camera


class Utilities():

#_______________________________________________________#
    #Mask
#---------------------------------------------------#

    @staticmethod
    def createHsvMask(img, minThresh, maxThresh):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, minThresh, maxThresh)
        count = cv2.countNonZero(mask)
        return count, mask

    @staticmethod
    def createGreyMask(img, minThresh, maxThresh):
        BGR = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(grey, minThresh, maxThresh)
        count = cv2.countNonZero(mask)
        return count, mask

    @staticmethod
    def cleanMask(array, minSum):
        shape = array.shape
        for i in range(shape[0]):
            if (np.sum(array[i]) < minSum):
                array[i,:] = 0
        for i in range(shape[1]):
            if (np.sum(array[:,i]) < minSum):
                array[:,i] = 0
        return array
 
    @staticmethod
    def getMaskList(frameList, maskValsList, greyValsList, clean = True):
        maskList = []
        cam = -1
        for frame in frameList:
            mask = 0
            cam +=1
                        
            for maskVal in maskValsList:
                mask += 1

                count, hsvMask = Utilities.createHsvMask(frame, maskVal[0], maskVal[1])
                if clean == True:
                    hsvMask = Utilities.cleanMask(hsvMask, 1000)
                maskList.append((cam, mask))
                maskList.append(hsvMask)

            for greyVal in greyValsList:

                count, greyMask = Utilities.createGreyMask(frame, greyVal[0], greyVal[1])
                if clean == True:
                    greyMask = Utilities.cleanMask(greyMask, 1000)
                maskList.append((cam, mask))
                maskList.append(greyMask)
                
        return maskList

    @staticmethod
    def showFramePause(imgList):
        for item in imgList:
            if (type(item) == tuple):
                print "next frame"
            else:
                esc = False
                while esc == False:
                    cv2.imshow("frame", item)
                    k = cv2.waitKey(30) & 0xff
                    if k == 27:
                        esc = True

#_______________________________________________________#
    #Array Manipulation & Math
#---------------------------------------------------#

    @staticmethod
    def cntrpnt2rect(x,y, rad):
        return (x+rad, y+rad, x-rad, y-rad)

#_______________________________________________________#
    #Frame/Image Editting
#---------------------------------------------------#

    @staticmethod
    def removeAllRect(image, matrix):
        newImage = image
        numRows = matrix.shape
        for i in range(numRows[0]):
            if not np.all(matrix[i,:] == 0):
                x = int(matrix[i,0])
                y = int(matrix[i,1])
                w = int(matrix[i,2])
                h = int(matrix[i,3])
                newImage = cv2.rectangle(newImage, (x,y), (x+w, y+h), (0,0,0), -1)
        return newImage

    @staticmethod
    def copyFrameList(frameList):
        newList = []
        length = len(frameList)
        for f in range(length):
            newFrame = np.copy(frameList[f])
            newList.append(newFrame)
        return newList
    
#_______________________________________________________#
    #Detector
#---------------------------------------------------#

    @staticmethod
    def getParams(object, detectorType):

        #Ground Robot Parameters
        if detectorType == 0:
            
            #Thresholds
            object.minThreshold = 0
            object.maxThreshold = 256

            #Filter by Color
            object.filterByColor = True
            object.blobColor = 255

            #Filter by Area
            object.filterByArea = True
            object.minArea = 100
            object.maxArea = 1500

            #Filter by Circularity
            object.filterByCircularity = False
            object.minCircularity = 0.1

            #Filter by Convexity
            object.filterByConvexity = False
            object.minConvexity = 0.5

            #Filter by Inertia
            object.filterByInertia = False

        #Obstacle Parameters
        #elif detectorType == 1:
        return

    @staticmethod
    def blobSearch(maskList, detector):
        boolList = [True, True, True, False, True]
        totItems = len(maskList)
        arrayList = []
            
        for m in range(totItems):
            detect = True
            if type(maskList[m]) == tuple:
                tup = maskList[m]
                cam = tup[0]
                color = tup[1]

            else:           
                while detect == True:
                    
                    mask = maskList[m]
                    obj = detector.detect(mask)
                    if not (obj == []):
                        x = int(obj[0].pt[0])
                        y = int(obj[0].pt[1])
                        r = int(obj[0].size/2)
                        #print "area", obj[0].area
                        if r < 15:
                            r += 25
                            
                        newArray = [cam,x,y,r,0,0,color]
                        arrayList.append(newArray)
                        mask = cv2.circle(mask, (x,y), r+25, (0,0,0), -1)
                    else:
                        detect = False
                        
        return arrayList, boolList




