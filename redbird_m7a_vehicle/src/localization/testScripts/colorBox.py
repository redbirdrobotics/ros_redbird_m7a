import cv2
import numpy as np

def basicColorMatch(img, minPix):
    detect = False
    xyArray = np.transpose(np.nonzero(img))
    shape = xyArray.shape
    if shape[0] >= minPix:
        detect = True
        bBox = cv2.boundingRect(xyArray)
    return bBox

tester = np.eye(6)
bobox = basicColorMatch(tester, 6)
print bobox
print bobox[0]
