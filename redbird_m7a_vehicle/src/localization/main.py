import cv2
import numpy as np
import time
from redbird import Robot
from redbird import Utilities
from redbird import Camera

#INITIALIZERS

#CAMERAS
#temp Height = 1.07
cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 0))
cam0.create_angleAxis()
##cam1 = Camera(1, (1280, 720), 60, (130, 90), (120, 45))
##cam1.create_angleAxis()
##cam2 = Camera(2, (1280, 720), 60, (130, 90), (240, 45))
##cam2.create_angleAxis()
camList = [cam0]

#ROBOTS
Marky = Robot(1)
Ricky = Robot(2)
Danny = Robot(3)
Terry = Robot(4)
Mikey = Robot(5)
Davey = Robot(6)
Timmy = Robot(7)
Tommy = Robot(8)
Joey = Robot(9)
Robby = Robot(10)
Johnny = Robot(11)
Brian = Robot(12)
Willy = Robot(13)
Benny = Robot(14)
roboList = [Marky, Ricky, Danny, Terry, Mikey, Davey, Timmy, Tommy, Joey, Robby, Johnny, Brian, Willy, Benny]

#DETECTORS

#HAAR
grndLBP1 = cv2.CascadeClassifier('address')
cascadeList = [grndLBP1]

#BLOB
greenMaskVals = np.array([[55,45,60],[85,230,230]])
redMaskVals = np.array([[163,182,136],[180,255,255]])
greyMaskVals = (145,175)
maskValsList = [greenMaskVals]

robotParams = cv2.SimpleBlobDetector_Params()
Utilities.getParams(robotParams, 0)
groundRobot = cv2.SimpleBlobDetector_create(robotParams)
blobList = [groundRobot]

#dataArray Format: cam(cam), coords(x,y), radius(r), vector(vx,vy), color(color) 

while True:

    #Get New Frames
    showingFrameList = Camera.getFrameList(camList)
    workingFrameList = Utilities.copyFrameList(showingFrameList)
    maskList = Utilities.getMaskList(workingFrameList, maskValsList, greyMaskVals, 0.75)
    
    #Search ROIs
    newmaskList = Robot.ROIsearch(roboList, maskList)
        
    #Search Whole Frame
    xyrcList, colList = Utilities.blobSearch(newmaskList, groundRobot)
    Robot.listUpdate(0, roboList, xyrcList, colList, maskValsList, blobList)
    
    #For Testing
    showingFrame = Robot.circleFound(showingFrameList[0], roboList)
    esc = Camera.showFrame(showingFrame, "cam0")   
    if esc ==True:
        break
    
cam0.detach()
