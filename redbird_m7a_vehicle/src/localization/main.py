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
Marky = Robot(0)
Ricky = Robot(1)
Danny = Robot(2)
Terry = Robot(3)
Mikey = Robot(4)
Davey = Robot(5)
Timmy = Robot(6)
Tommy = Robot(7)
Joey = Robot(8)
Robby = Robot(9)
Johnny = Robot(10)
Brian = Robot(11)
Willy = Robot(12)
Benny = Robot(13)
roboList = [Marky, Ricky, Danny, Terry, Mikey, Davey, Timmy, Tommy, Joey, Robby, Johnny, Brian, Willy, Benny]

#DETECTORS

#HAAR
grndLBP1 = cv2.CascadeClassifier('address')
cascadeList = [grndLBP1]

#BLOB
greenMaskVals = np.array([[75,56,61],[86,178,114]])
redMaskVals = np.array([[163,182,136],[180,255,255]])
greyMaskVals = (230,255)
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
    maskList = Utilities.getMaskList(workingFrameList, maskValsList, greyMaskVals, 0.15)

    #Search ROIs
    newmaskList = Robot.ROIsearch(roboList, maskList)
        
    #Search Whole Frame
    xyrcList, colList = Utilities.blobSearch(newmaskList, groundRobot)
    Robot.listUpdate(roboList, xyrcList, colList, maskValsList, blobList)
    
    #For Testing
    showingFrame = Robot.circleFound(showingFrameList[0], roboList)
    esc = Camera.showFrame(showingFrame, "cam0")   
    if esc ==True:
        break
    
cam0.detach()
