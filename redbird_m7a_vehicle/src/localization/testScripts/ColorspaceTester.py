import cv2
import numpy as np
from redbird import Camera

cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 45))
mask = np.array([[55,45,60],[88,230,230]])

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

while True:
   frame = cam0.getFrame()
   ret, newMask = createMask(frame, mask[0], mask[1], 100)
   cleanMask = cleanMask(newMask, 1000)
   esc = Camera.showFrame(cleanMask, "mask1")
   if esc == True:
       break
cam0.detach()
    



