import cv2
import numpy as np
from redbird import Camera

frame = cv2.imread("C:/Users/Alex/Desktop/Robotics/Project/ros_redbird_m7a/redbird_m7a_vehicle/src/localization/testImages/gridC.jpg")
frameA = cv2.resize(frame, (1280, 720))
lineMaskVals = np.array([[80, 0, 207], [110, 32, 255]])

def hsvMask(img, minThresh, maxThresh):
    hsv = cv2.cvtColor(img, cv2.cv.CV_BGR2HSV)
    mask = cv2.inRange(hsv, minThresh, maxThresh)
    count = cv2.countNonZero(mask)
    return count, mask

def detectLines(img, rho, theta, minLen):
    lines = cv2.HoughLines(hsv, rhoArg, np.pi/thetaArg, minLengthArg)
    if not (lines == None):
        for rho, theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            xN = a*rho
            yN = b*rho
            
            xA = int(x0 + 2000*(-b))
            yA = int(y0 + 2000*(a))
            xB = int(x0 - 2000*(-b))
            yB = int(y0 - 2000*(a))
    return xA, yA, xB, yB

def contourSearch(img, minContours):
    detect = 0
    
    contourDataList = cv2.findContours(img, cv2.cv.CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE)
    contourGroupList = contourDataList[1]

    if contourGroupList == []:
        print "is empty"
        return None, None, None

    else:
        contourList = contourGroupList[0]
        peri = cv2.arcLength(contourList, True)
        approx = cv2.approxPolyDP(contourList, 0.04*peri, True)
        numSides = len(approx)

        area = cv2.contourArea(contourList)
        print dataCollect(numSides, peri, area)
        
        if numSides >= minContours:
            detect = 1

    return contourList, detect, dataArray

tabulaRasa = np.zeros((720, 1280, 3),np.uint8)

while True:

    ret, mask = hsvMask(frameA, lineMaskVals[0], lineMaskVals[1])
    escB = Camera.showFrame(mask, 'Mask')
    
    contourDataList = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
    if not contourDataList == []:
        
        contourGroupList = contourDataList[0]
        
        contourList = contourGroupList[0]
        print 'contourGroupList has', len(contourGroupList), 'members'

        vetA = [contour for contour in contourGroupList if (contour.shape)[0] >= 10]
        print 'ProccessA has', len(vetA), 'members'
        print vetA

        lineList = []
        for contour in vetA:
            peri = cv2.arcLength(vetA, True)
            polyline = cv2.approxPolyDP(vetA, 0.04*peri, False)
            lineList.append(polyline)

        print lineList
        
        

        #cv2.drawContours(tabulaRasa, [np.array(contourList).reshape((-1,1,2)).astype(np.int32)], -1, (255,255,255), 3)
        cv2.drawContours(tabulaRasa, vetA, -1, (255,255,255), 1)
        
        escA = Camera.showFrame(tabulaRasa, "Lines")

        break

    
        if escA == True or escB == True:
            break

    print 'loop'
