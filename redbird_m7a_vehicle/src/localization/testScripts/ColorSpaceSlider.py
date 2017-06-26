import cv2
import numpy as np
from redbird import Camera

cam1 = Camera(1, (1280,720), 60, (130,90), (0,45))
#win = np.zeros((480, 720, 3))


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

cv2.namedWindow('Slider')
cv2.resizeWindow('Slider', 720, 780)

cv2.createTrackbar('Alo_H', 'Slider', 50, 180, nothing)
cv2.createTrackbar('Alo_S', 'Slider', 50, 255, nothing)
cv2.createTrackbar('Alo_V', 'Slider', 50, 255, nothing)
cv2.createTrackbar('Ahi_H', 'Slider', 80, 180, nothing)
cv2.createTrackbar('Ahi_S', 'Slider', 80, 255, nothing)
cv2.createTrackbar('Ahi_V', 'Slider', 80, 255, nothing)

cv2.createTrackbar('Blo_H', 'Slider', 50, 180, nothing)
cv2.createTrackbar('Blo_S', 'Slider', 50, 255, nothing)
cv2.createTrackbar('Blo_V', 'Slider', 50, 255, nothing)
cv2.createTrackbar('Bhi_H', 'Slider', 80, 180, nothing)
cv2.createTrackbar('Bhi_S', 'Slider', 80, 255, nothing)
cv2.createTrackbar('Bhi_V', 'Slider', 80, 255, nothing)


while True:

    frame = cam1.getFrame()

    AloH = cv2.getTrackbarPos('Alo_H', 'Slider' )
    AloS = cv2.getTrackbarPos('Alo_S', 'Slider' )
    AloV = cv2.getTrackbarPos('Alo_V', 'Slider' )
    AhiH = cv2.getTrackbarPos('Ahi_H', 'Slider' )
    AhiS = cv2.getTrackbarPos('Ahi_S', 'Slider' )
    AhiV = cv2.getTrackbarPos('Ahi_V', 'Slider' )

    BloH = cv2.getTrackbarPos('Blo_H', 'Slider' )
    BloS = cv2.getTrackbarPos('Blo_S', 'Slider' )
    BloV = cv2.getTrackbarPos('Blo_V', 'Slider' )
    BhiH = cv2.getTrackbarPos('Bhi_H', 'Slider' )
    BhiS = cv2.getTrackbarPos('Bhi_S', 'Slider' )
    BhiV = cv2.getTrackbarPos('Bhi_V', 'Slider' )

    

    ret, maskA = createMask(frame, np.array([AloH, AloS, AloV]), np.array([AhiH, AhiS, AhiV]), 100)
    ret, maskB = createMask(frame, np.array([BloH, BloS, BloV]), np.array([BhiH, BhiS, BhiV]), 100)

    newMask = cv2.add(maskA, maskB)

    #win[:] = [0,0,0]

    escA = Camera.showFrame(newMask, "Mask")
    #escB = Camera.showFrame(win, "Slider")
    if escA == True:
        break

cam1.detach()
    










    
    
