import cv2 
import numpy as np
import time


def getParameters(object, detectorType):

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
            object.minArea = 80

            object.filterByCircularity = False
            object.minCircularity = 0.1

            object.filterByConvexity = False
            object.minConvexity = 0.5

            object.filterByInertia = False

        #Obstacle Parameters
        #elif detectorType == 1:
        return

def createMask(img, minThresh, maxThresh, minPixel):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, minThresh, maxThresh)
        count = cv2.countNonZero(mask)

        if (count > minPixel):
            ret = True
        else:
            ret = False
        return ret, mask

def cleanMask(array, minSum):
        shape = array.shape
        for i in range(shape[0]):
            if (np.sum(array[i]) < minSum):
                array[i,:] = 0
        for i in range(shape[1]):
            if (np.sum(array[:,i]) < minSum):
                array[:,i] = 0
        return array 

#Camera
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 780)
cap.set(cv2.CAP_PROP_FPS, 60)



#Mask
greenMaskVals = np.array([[55,45,60],[85,230,230]])

#Blob
robotParams = cv2.SimpleBlobDetector_Params()
getParameters(robotParams, 0)
groundRobot = cv2.SimpleBlobDetector_create(robotParams)


ret, frame = cap.read()
esc, mask = createMask(frame, greenMaskVals[0], greenMaskVals[1], 100)
mask = cleanMask(mask, 100)
loop = 0

while True:        
        keypoints = groundRobot.detect(mask)
        mask = cv2.drawKeypoints(mask, keypoints, (0,0,225), flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow('mask' ,mask)
        if cv2.waitKey(1) & 0xFF ==ord('q'):
                break

        x = int(keypoints[0].pt[0])
        y = int(keypoints[0].pt[1])
        r = int(keypoints[0].size/2)
        b = 25
                
        cv2.circle(mask, (x,y), r+b, (0,0,0), -1)
        loop += 1
        print loop

cap.release()
cv2.destroyAllWindows()
