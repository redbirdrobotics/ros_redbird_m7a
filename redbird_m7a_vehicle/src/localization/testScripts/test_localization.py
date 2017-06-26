import numpy as np
import cv2
from redbird import Camera
from redbird import Utilities
import time

#green = False
#red = False

#Initialize Camera
cam0 = Camera(1, (1280, 780), 60, (130, 90), (0, 90))
cam0.create_angleAxis()

#Initialize Blob Detector
params = cv2.SimpleBlobDetector_Params()

#Change Thresholds
params.minThreshold = 0;
params.maxThreshold = 255;

#Filter by Color
params.blobColor = 255

#Filter by Area
params.filterByArea = True
params.minArea = 100

#Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.1

#Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.5

#Filter by Inertia
params.filterByInertia = False

detector = cv2.SimpleBlobDetector_create(params)

#Masking Variables
min_pixel = 100
thresh_min1 = np.array([55,50,60])
thresh_max1 = np.array([85,255,255])

def test_color(img, threshold_min, threshold_max, min_pixel):
    ret = False
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, threshold_min, threshold_max)
    count = cv2.countNonZero(mask)
    
    if(count > min_pixel):
        ret = True
        
    return ret, mask
  

while True:
    
    frame = cam0.getFrame()
    
    ret, green = test_color(frame, thresh_min1, thresh_max1, min_pixel)
    green_editted = Utilities.zero_ifLessThan(green, 150)
    keypoints = detector.detect(green_editted)

    if keypoints == []:
        print "empty array"

    print keypoints
    
    length = len(keypoints)
    for i in range(length):
        print i
        print keypoints[i].pt
        x = int(keypoints[i].pt[0])
        y = int(keypoints[i].pt[1])
        #dist = Utilities.get_dis(cam0.xaxis, cam0.yaxis, (x, y), 1.1176)
        #print dist
        
    newFrame = cv2.drawKeypoints(frame, keypoints, (0,0,255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    esc = cam0.showFrame(newFrame, 'Cam0')
    if esc == True:
        break

cam0.detach()

