import cv2
import numpy as np
from redbird import*
#from RedRobotLib import*

#INITIALIZERS

#CAMERAS TO BE REPLACES BY GSTREAMER FUNCTION
cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 53.7))
camList = [cam0]

#RED ROBOTS
DareDevil = RedRobot(0)
DeadPool = RedRobot(1)
Elmo = RedRobot(2)
HellBoy = RedRobot(3)
Flash = RedRobot(4)
robotList = [DareDevil, DeadPool, Elmo, HellBoy, Flash]

#THRESHOLD
redVals = np.array([[165,150,150],[180,240,200]])

#BLOB DETECTOR
RedRobotParams = cv2.SimpleBlobDetector_Params()
Utilities.getParams(RedRobotParams, 0)
detector = cv2.SimpleBlobDetector_create(RedRobotParams)

#LISTS
foundList = []
unfoundList = []

frameList = []
maskList = []
dataList = []

while True:

    #Get Mav_ROS Node Data
    #Function that would get current position of quad
    RedRobot.listcvt2meters(0, 0, 1.524, foundList, camList)

    #Get Gstream Node Data
    #Replace following function with function that gets image list
    Camera.getFrameList(camList, frameList)
    
    Utilities.getMaskList(frameList, redVals, maskList)
    #Get Landmark Node Data

    #Search ROI
    RedRobot.ROIsearch(foundList, maskList, detector)
    RedRobot.sortFound(robotList, foundList, unfoundList)
    
    #Search Whole Frame
    Utilities.blobSearch(maskList, detector, dataList, unfoundList)
    RedRobot.listUpdate(foundList, unfoundList, dataList, camList)
    RedRobot.sortFound(robotList, foundList, unfoundList)

    #For Testing
    frame = Utilities.circleFound(frameList[0], foundList)
    #Function to remove landmarks from node data
    esc = Camera.showFrame(frame, 'frame')
    if esc == True:
        break

cam0.detach()








    
