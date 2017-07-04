import cv2
import numpy as np
from redbird import Camera

frame = cv2.imread("C:/Users/Alex/Desktop/Robotics/Project/ros_redbird_m7a/redbird_m7a_vehicle/src/localization/testImages/gridC.jpg",1)
frameA = cv2.resize(frame, (1280, 720))
frameB = np.copy(frameA)

def createLineBox(img, x1, x2, y1, y2, r):
    contour = np.array([[x1, y1],[x2,y2]])
    rect = cv2.minAreaRect(contour)
    
    #Add height to line to make into rect and reassemble
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

    img = createLineBox(frameA, 800, 1100, 300, 500, 10)

    escA = Camera.showFrame(img, "Box")
    if escA == True:
        break
