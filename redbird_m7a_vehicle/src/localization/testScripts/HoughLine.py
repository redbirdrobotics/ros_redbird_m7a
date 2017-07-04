import cv2
import numpy as np
from redbird import Camera

#cam1 = Camera(1, (1280,720), 60, (130,90), (0,45))
frame = cv2.imread("C:/Users/Alex/Desktop/Robotics/Project/ros_redbird_m7a/redbird_m7a_vehicle/src/localization/testImages/gridC.jpg",1)
frameA = cv2.resize(frame, (1280, 720))
frameB = np.copy(frameA)
cv2.namedWindow('Slider')
cv2.resizeWindow('Slider', 720, 780)

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

def createLineBox(img, x1, x2, y1, y2, r):
    contour = np.array([[x1, y1],[x2,y2]])
    rect = cv2.minAreaRect(contour)
    
    #disect rect add height to line to make into actual rectangle and reassemble
    #rect Format: ((x,y), (w,h), theta)
    x,y = rect[0]
    w,h = rect[1]
    theta = rect[2]
    newRect = ((x,y),(w,h+r),theta)
    
    box = cv2.boxPoints(newRect)
    box = np.int0(box)
    boxImg = cv2.drawContours(img, [box], 0, (0,0,0), -1)
    return boxImg


#TRACKBARS
#--------------#

#Hough Arguments#
cv2.createTrackbar('rho', 'Slider', 1, 50, nothing)
cv2.createTrackbar('theta', 'Slider', 180, 180, nothing)
cv2.createTrackbar('minLength', 'Slider', 1, 1200, nothing)

#Grey Mask#
#cv2.createTrackbar('greyMin', 'Slider', 50, 180, nothing)
#cv2.createTrackbar('greyMax', 'Slider', 100, 180, nothing)

#HSV Mask#
cv2.createTrackbar('hMin', 'Slider', 1, 179, nothing)
cv2.createTrackbar('hMax', 'Slider', 180, 180, nothing)
cv2.createTrackbar('sMin', 'Slider', 1, 254, nothing)
cv2.createTrackbar('sMax', 'Slider', 254, 255, nothing)
cv2.createTrackbar('vMin', 'Slider', 1, 254, nothing)
cv2.createTrackbar('vMax', 'Slider', 254, 255, nothing)

#Weight of Masks#
#cv2.createTrackbar('beta', 'Slider', 100, 100, nothing)

#Switch
cv2.createTrackbar('Switch', 'Slider', 0, 1, nothing)



#Escape Condition
escA = False


#MAIN LOOP#
#------------#
while True:
    
    rhoArg = cv2.getTrackbarPos('rho', 'Slider')
    thetaArg = cv2.getTrackbarPos('theta', 'Slider')
    minLengthArg = cv2.getTrackbarPos('minLength', 'Slider')
    hMin = cv2.getTrackbarPos('hMin', 'Slider')
    hMax = cv2.getTrackbarPos('hMax', 'Slider')
    sMin = cv2.getTrackbarPos('sMin', 'Slider')
    sMax = cv2.getTrackbarPos('sMax', 'Slider')
    vMin = cv2.getTrackbarPos('vMin', 'Slider')
    vMax = cv2.getTrackbarPos('vMax', 'Slider')
    switch = cv2.getTrackbarPos('Switch', 'Slider')

    count, hsv = hsvMask(frameA, np.array([hMin,sMin,vMin]), np.array([hMax,sMax,vMax]))

    lines = cv2.HoughLines(hsv, rhoArg, np.pi/thetaArg, minLengthArg)
    if switch == 1:
        if not (lines == None):
            print "found a line"
            for rho, theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                
                x1 = int(x0 + 2000*(-b))
                y1 = int(y0 + 2000*(a))
                x2 = int(x0 - 2000*(-b))
                y2 = int(y0 - 2000*(a))

            #linesImg = cv2.line(frameB,(x1,y1),(x2,y2),(0,0,255),2)
            frameB = createLineBox(frameB, x1, x2, y1, y2, 10)
            escA = Camera.showFrame(frameB, "Lines")
    
    escB = Camera.showFrame(hsv, "HSV")
    if (escA == True or escB == True):
        break

#cam1.detach()
    










    
    
