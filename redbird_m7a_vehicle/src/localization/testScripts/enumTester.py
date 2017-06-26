import cv2
import math
import numpy as np
from enum import Enum

class Searcher():
    
    def __init__(self):
            
        #Masks
        self.greenMaskVals = np.array([[55,45,60], [85,230,230]])
        self.redMaskVals = np.array([[0,0,0], [0,0,0]])

        #Blob Parameters
        robotParams = cv2.SimpleBlobDetector_Params()
        self.getParams(robotParams, 0)
        gr
        self.groundRobotDetector = = cv2.SimpleBlobDetector_create(robotParams)
 
        #CASE: LBP Cascade
        grndLBP = cv2.CscadeClassifier('address')
        
        return

    
