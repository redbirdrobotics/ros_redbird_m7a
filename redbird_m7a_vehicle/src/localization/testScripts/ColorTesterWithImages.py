import cv2
import numpy as np
import os
from redbird import Camera
from redbird import Robot

imgA = cv2.imread("C:/Users/Alex/Desktop/IARC/testImages/greenA.jpg", 1)
imgB = cv2.imread('C:/Users/Alex/Desktop/IARC/testImages/greenB.jpg', 1)
imgC = cv2.imread('C:/Users/Alex/Desktop/IARC/testImages/greenC.jpg', 1)
imageList = [imgA, imgB, imgC]

mask = np.array([[55,45,60],[88,230,230]])

robA = Robot()
robB = Robot()
robC = Robot()
robD = Robot()
robList = [robA, robB, robC, robD]

dataList = [[1,70,75,50,6,6,0], [1,2,45,66,6,0,1], [0,0,0,0,0,0,0], [0,1,1,20,1,1,1]]

Robot.update(1, robList, dataList, [True, True, True, False, True])

ROIList = Robot.frameList2ROIList(robList, imageList)

while True:

##    escA = Camera.showFrame(imgA, "ROI1")
##    escB = Camera.showFrame(imgB, "ROI2")
##    escC = Camera.showFrame(imgC, "ROI2")

    escA = Camera.showFrame(ROIList[0], "ROI1")
##    escB = Camera.showFrame(ROIList[1], "ROI2")
##    escC = Camera.showFrame(ROIList[2], "ROI2")
##    escD = Camera.showFrame(ROIList[3], "ROI3")
   
    if escA == True or escB == True or escC == True:
        break
