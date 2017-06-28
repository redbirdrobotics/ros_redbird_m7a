import cv2
import numpy as np
from redbird import Camera

cam1 = Camera(1, (1280,720), 60, (130,90), (0,45))
#win = np.zeros((480, 720, 3))
frame = cv2.imread("C:/Users/Alex/Desktop/Robotics/Project/ros_redbird_m7a/redbird_m7a_vehicle/src/localization/testImages/gridBotC.jpg",1)
newFrame = cv2.resize(frame, (1280, 720))

def nothing(x):
    pass

def createMask(img, minThresh, maxThresh, minPixel):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, minThresh, maxThresh)
    count = cv2.countNonZero(mask)

    if (count > minPixel):
        ret = True
    else:
        ret = False
    return ret, mask

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

cv2.createTrackbar('xA', 'Slider', 0, 1279, nothing)
cv2.createTrackbar('xB', 'Slider', 1280, 1280, nothing)
cv2.createTrackbar('yA', 'Slider', 0, 719, nothing)
cv2.createTrackbar('yB', 'Slider', 720, 720, nothing)


while True:

    #frame = cam1.getFrame()

    xA = cv2.getTrackbarPos('xA', 'Slider' )
    xB = cv2.getTrackbarPos('xB', 'Slider' )
    yA = cv2.getTrackbarPos('yA', 'Slider' )
    yB = cv2.getTrackbarPos('yB', 'Slider' )

    imgROI = createROI(newFrame, xA, xB, yA, yB)

    #newMask = cv2.add(maskA, maskB)

    #win[:] = [0,0,0]

    escA = Camera.showFrame(imgROI, "ROI")
    #escB = Camera.showFrame(win, "Slider")
    if escA == True:
        break

cam1.detach()
    










    
    
