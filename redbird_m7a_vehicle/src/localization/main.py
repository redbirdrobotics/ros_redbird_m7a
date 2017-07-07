import cv2
import numpy as np
import time
from redbird import Robot
from redbird import Utilities
from redbird import Camera
from redbird import Landmark

#INITIALIZERS

#CAMERAS
#temp Height = 1.07
cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 53.7))
cam0.create_angleAxis()
##cam1 = Camera(1, (1280, 720), 60, (130, 90), (120, 45))
##cam1.create_angleAxis()
##cam2 = Camera(2, (1280, 720), 60, (130, 90), (240, 45))
##cam2.create_angleAxis()
camList = [cam0]

#ROBOTS
Quad = Robot(100)
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

#LANDMARKS
goalLine = Landmark()

#DETECTORS

#HAAR
grndLBP = cv2.CascadeClassifier('address')
cascadeList = [grndLBP]

#HOUGH
goalVals = np.array([[79,33,66],[100,111,135]])
hsvHoughMaskList = [goalVals]

#BLOB
greenVals = np.array([[70,81,78],[94,241,154]])
redVals = np.array([[163,182,136],[180,255,255]])
whiteVals = np.array([[230],[255]])
hsvBlobMaskList = [greenVals]
greyBlobMaskList = []

robotParams = cv2.SimpleBlobDetector_Params()
Utilities.getParams(robotParams, 0)
groundRobot = cv2.SimpleBlobDetector_create(robotParams)
blobList = [groundRobot]

#dataArray Format: cam(cam), coords(x,y), radius(r), vector(vx,vy), color(color) 

while True:
    
    #Get Node Data
    Quad.quadUpdate(0, 0, 1.524)
    Robot.listConversion(Quad, roboList, camList)

    #Get New Frames
    showingFrameList = Camera.getFrameList(camList)
    workingFrameList = Utilities.copyFrameList(showingFrameList)
    robotMaskList = Utilities.getMaskList(workingFrameList, hsvBlobMaskList, greyBlobMaskList)
    goalMaskList = Utilities.getMaskList(workingFrameList, hsvHoughMaskList, [])
    #Utilities.showFramePause(robotMaskList)

    #Search For Landmarks
    goalLine.detectGoalLine(goalMaskList)
    goalLine.remove(robotMaskList, 15)
    
    #Search Robot ROIs
    newmaskList = Robot.ROIsearch(roboList, robotMaskList)
    
    
    #Search Whole Frame
    xyrcList, colList = Utilities.blobSearch(newmaskList, groundRobot)
    Robot.listUpdate(roboList, xyrcList, colList, hsvBlobMaskList, blobList)
    
    #For Testing
    showingFrame = Robot.circleFound(showingFrameList[0], roboList)
    showingFrame = goalLine.drawLine(showingFrame, 15)
    esc = Camera.showFrame(showingFrame, "cam0")   
    if esc ==True:
        break
    
cam0.detach()
