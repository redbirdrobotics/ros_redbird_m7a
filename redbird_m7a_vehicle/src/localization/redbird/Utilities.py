import rospy
import cv2
import numpy as np


class Utilities():

    @staticmethod
    def emptyList(anyList):
        maxElem = len(anyList)
        del anyList[0:maxElem]

#_______________________________________________________#
    #Mask
#---------------------------------------------------#

    @staticmethod
    def createHSVMask(img, minThresh, maxThresh):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, minThresh, maxThresh)
        count = cv2.countNonZero(mask)
        return mask

    @staticmethod
    def getMaskList(frameList, maskVals, maskList):
        Utilities.emptyList(maskList)
        for frame in frameList:
            hsvMask = Utilities.createHSVMask(frame, maskVals[0], maskVals[1])
            maskList.append(hsvMask)
        return

#_______________________________________________________#
    #User Interface
#---------------------------------------------------#

    @staticmethod
    def circleFound(image, foundList):
        for robot in foundList:
            x, y = robot.coords
            vX, vY = robot.vector
            r = robot.radius
            mx, my = robot.mcoords
            mx = round(mx, 2)
            my = round(my, 2)

            image = cv2.circle(image, (x,y), r, (0,0,0), 1)
            image = cv2.putText(image, ("%s"% robot.ident), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
            #image = cv2.putText(image, ("mx: %s Vx %s" % (mx, vX)), (x-10, y+15), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
            #image = cv2.putText(image, ("my: %s Vy %s" % (my, vY)), (x-10, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
            #image = cv2.line(image, (x,y), (x + vX, y + vY), (0,0,0), 5)
        return image

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
            object.minArea = 50
            object.maxArea = 6000

            #Filter by Circularity
            object.filterByCircularity = False
            object.minCircularity = 0.1

            #Filter by Convexity
            object.filterByConvexity = False
            object.minConvexity = 0.5

            #Filter by Inertia
            object.filterByInertia = False
        return

    @staticmethod
    def blobSearch(maskList, detector, dataList, unfoundList):

        if not unfoundList:
            # print 'skip blob search no unfound'
            return

        Utilities.emptyList(dataList)
        cam = 0
        detect = True

        while detect == True:
            keypoints = detector.detect(maskList[cam])

            if not keypoints:
                cam +=1

                if cam > (len(maskList) - 1):
                    detect = False
                    # print 'No blobs detected'
                    return

            else:
                x = int(keypoints[0].pt[0])
                y = int(keypoints[0].pt[1])
                r = int(keypoints[0].size/2)

                if r<35:
                    r+= 35
                # print'Found Blob at ', x,y,r
                dataList.append([cam,x,y,r,0,0])
                maskList[cam] = cv2.circle(maskList[cam], (x,y), r, (0,0,0), -1)
        return
