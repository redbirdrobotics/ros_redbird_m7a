import cv2
import numpy as np
from redbird import Camera

cam1 = Camera(1, (1280,720), 60, (130,90), (0,45))
#win = np.zeros((480, 720, 3))


def nothing(x):
    pass

def createMask(img, minThresh, maxThresh, minPixel):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, minThresh, maxThresh)
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

cv2.namedWindow('Slider')
cv2.resizeWindow('Slider', 720, 780)

cv2.createTrackbar('Low', 'Slider', 50, 180, nothing)
cv2.createTrackbar('High', 'Slider', 50, 255, nothing)


while True:

    frame = cam1.getFrame()

    Low = cv2.getTrackbarPos('Low', 'Slider' )
    High = cv2.getTrackbarPos('High', 'Slider' )
    
    

    ret, maskA = createMask(frame, np.array([Low]), np.array([High]), 100)
  

    #win[:] = [0,0,0]

    escA = Camera.showFrame(maskA, "Mask")
    #escB = Camera.showFrame(win, "Slider")
    if escA == True:
        break

cam1.detach()
    










    
    
