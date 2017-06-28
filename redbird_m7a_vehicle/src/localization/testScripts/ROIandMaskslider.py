import cv2
import numpy as np
from redbird import Camera

#cam1 = Camera(1, (1280,720), 60, (130,90), (0,45))
frame = cv2.imread("C:/Users/Alex/Desktop/Robotics/Project/ros_redbird_m7a/redbird_m7a_vehicle/src/localization/testImages/gridBotC.jpg",1)
frameA = cv2.resize(frame, (1280, 720))
frameB = np.copy(frameA)

def nothing(x):
    pass

def hsvMask(img, minThresh, maxThresh):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, minThresh, maxThresh)
    count = cv2.countNonZero(mask)
    return count, mask

def greyMask(img, minThresh, maxThresh):
    BGR = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    grey = cv2.cvtColor(BGR, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(grey, minThresh, maxThresh)
    count = cv2.countNonZero(mask)
    return count, mask

def cleanMask(img, minSum):
    shape = img.shape
    for i in range(shape[0]):
        if (np.sum(img[i]) < minSum):
            img[i,:] = 0
    for i in range(shape[1]):
        if (np.sum(img[:,i]) < minSum):
            img[:,i] = 0
    return img

def createROI(img, xA, xB, yA, yB):
    return img[yA:yB, xA:xB]

cv2.namedWindow('Slider')
cv2.resizeWindow('Slider', 720, 780)

#TRACKBARS
#--------------#

#HSV Mask#
cv2.createTrackbar('hMin', 'Slider', 1, 179, nothing)
cv2.createTrackbar('hMax', 'Slider', 180, 180, nothing)
cv2.createTrackbar('sMin', 'Slider', 1, 254, nothing)
cv2.createTrackbar('sMax', 'Slider', 254, 255, nothing)
cv2.createTrackbar('vMin', 'Slider', 1, 254, nothing)
cv2.createTrackbar('vMax', 'Slider', 254, 255, nothing)

#Grey Mask#
cv2.createTrackbar('greyMin', 'Slider', 0, 255, nothing)
cv2.createTrackbar('greyMax', 'Slider', 254, 255, nothing)

#ROI#
cv2.createTrackbar('xA', 'Slider', 0, 1279, nothing)
cv2.createTrackbar('xB', 'Slider', 1280, 1280, nothing)
cv2.createTrackbar('yA', 'Slider', 0, 719, nothing)
cv2.createTrackbar('yB', 'Slider', 720, 720, nothing)

#Weight of Masks#
cv2.createTrackbar('alpha', 'Slider', 100, 100, nothing)

while True:
    hMin = cv2.getTrackbarPos('hMin', 'Slider')
    hMax = cv2.getTrackbarPos('hMax', 'Slider')
    sMin = cv2.getTrackbarPos('sMin', 'Slider')
    sMax = cv2.getTrackbarPos('sMax', 'Slider')
    vMin = cv2.getTrackbarPos('vMin', 'Slider')
    vMax = cv2.getTrackbarPos('vMax', 'Slider')
    gMin = cv2.getTrackbarPos('greyMin', 'Slider')
    gMax = cv2.getTrackbarPos('greyMax', 'Slider')
    xA = cv2.getTrackbarPos('xA', 'Slider')
    xB = cv2.getTrackbarPos('xB', 'Slider')
    yA = cv2.getTrackbarPos('yA', 'Slider')
    yB = cv2.getTrackbarPos('yB', 'Slider')
    alphaPercent = cv2.getTrackbarPos('alpha', 'Slider')

    alpha = alphaPercent/float(100)
    count, hsv = hsvMask(frameA, np.array([hMin,sMin,vMin]), np.array([hMax,sMax,vMax]))
    count, grey = greyMask(frameB, gMin, gMax)
    wMask = cv2.addWeighted(hsv, alpha, grey, 1-alpha, 0.0)
    imgROI = createROI(wMask, xA, xB, yA, yB)

    

    escA = Camera.showFrame(imgROI, "ROI")
    if escA == True:
        break

#cam1.detach()
    










    
    
