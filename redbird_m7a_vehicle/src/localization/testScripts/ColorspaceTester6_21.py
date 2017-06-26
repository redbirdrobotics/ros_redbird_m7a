import cv2
import numpy as np
from redbird import Camera

cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 45))
#greenMask = np.array([[55,45,60],[80,230,230]])
redMask = np.array([[163,182,136],[180,255,255]])
#loRedMask = np.array([[0,190,190],[3,230,230]])
#upRedMask = np.array([[160,100,100],[180,255,255]])
    

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

while True:
   frame = cam0.getFrame()
   ret, loMask = createMask(frame, loRedMask[0], loRedMask[1], 100)
   ret, upMask = createMask(frame, upRedMask[0], upRedMask[1], 100)
   newMask = cv2.add(loMask, upMask)
   clnMask = cleanMask(newMask, 1000)
   esc = Camera.showFrame(clnMask, "mask1")
   if esc == True:
       break
cam0.detach()
    



