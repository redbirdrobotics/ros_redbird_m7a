import cv2
import numpy as np
from redbird import Camera

cam1 = Camera(1, (1280,720), 60, (130,90), (0,45))

def hsvMask(img, minThresh, maxThresh):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, minThresh, maxThresh)
    count = cv2.countNonZero(mask)
    return count, mask

def detectLines(img, rhoArg, thetaArg, minLen):
    lines = cv2.HoughLines(img, rhoArg, np.pi/thetaArg, minLen)
    if not (lines == None):
        lineVals = lines.shape
        for l in range(lineVals[0]):
            for rho, theta in lines[l]:
                a = np.cos(theta)
                b = np.sin(theta)
                xN = a*rho
                yN = b*rho
              
                if xN < 0:
                    xN = -xN
                    
                yN = b*rho
                if yN < 0:
                    yN = -yN
                
                xA = int(xN + 2000*(-b))
                yA = int(yN + 2000*(a))
                xB = int(xN - 2000*(-b))
                yB = int(yN - 2000*(a))
    else:
        xA = 0
        yA = 0
        xB = 0
        yB = 0
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

while True:

    frameA = cam1.getFrame()
    frameB = np.copy(frameA)

    count, mask = hsvMask(frameA, np.array([87,51,75]), np.array([98,94,90]))
    xA, yA, xB, yB = detectLines(mask, 1, 180, 50)
    cv2.line(frameB, (xA,yA),(xB,yB),(0,0,255), 15)
    #frameB = createLineBox(frameB, xA, yA, xB, yB, 10)

    esc = Camera.showFrame(frameB, "Line")
    if esc == True:
        break
    






    
    




