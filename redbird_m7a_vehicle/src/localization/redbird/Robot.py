import cv2
from Utilities import Utilities
import numpy as np
import time

             
class Robot():

#_______________________________________________________#
    #INITIALIZERS
#---------------------------------------------------#
    def __init__(self, num):
        self.ident = num
        self.cam = None
        self.coords = (0,0)
        self.radius = 0
        self.ROI = None
        self.vector = (0,0)
        self.color = 0
        self.blob = None
        self.lostNum = 0
        self.mcoords = (0,0)
        self.allData = [self.ident, self.cam, self.coords, self.radius, self.ROI, self.vector, self.color, self.blob, self.lostNum, self.mcoords]
        return

#_______________________________________________________#
    #GENERAL
#---------------------------------------------------#

    def wipeRobot(self):
        self.cam = None
        self.coords = (0,0)
        self.radius = 0
        self.ROI = None
        self.vector = (0,0)
        self.color = 0
        self.blob = None
        self.lostNum = 0
        self.mcoords = (0,0)
        return

    @staticmethod
    def listUnfound(objList):
        ret = False
        listUnfound = []
        lgthList = len(objList)
        
        for robot in objList:
            if all(c == 0 for c in robot.coords):
                listUnfound.append(robot.ident)            
        return listUnfound

    @staticmethod
    def printAllData(objList):
        for robot in objList:
            print robot.allData
#_______________________________________________________#
    #LOST NUMBER METHODS
#---------------------------------------------------#
    def setLostNum(self, lostNum):
        self.lostNum = lostNum
        return

    def incLostNum(self, threshold):
        lost = False
        self.lostNum = self.lostNum + 1
        if self.lostNum == threshold:
            lost = True
            self.ROI = None
            self.lostNum = 0
        return lost

    def correctLostNum(self):
        self.lostNum = 0

    def lostNumAsScalar(self):
        scalarVals = [1, 2, 2.5, 3.5, 4.5, 5.5]
        scalar = scalarVals[self.lostNum]
        return scalar
    
#_______________________________________________________#
    #COLOR METHODS
#---------------------------------------------------#
    def getColor(self):
        return self.color

    def updateColorProps(self, maskList, blobList):
        if self.color == 0:
            return
        
        elif self.color == 1:
            self.maskVals = maskList[0]
            self.blob = blobList[0]

        elif self.color == 2:
            self.maskVals = maskList[1]
            self.blob = blobList[0]

        elif self.color == 3:
            self.maskVals = maskList[2]
            self.blob = blobList[1]
        return
#_______________________________________________________#
    #ROI METHODS
#---------------------------------------------------#
    
    def createROI(self, x, y, r):
        if y-r == 0:
            y = 0
            
        if x-r == 0:
            x = 0

        self.ROI = np.array([y-r, y+r, x-r, x+r])
        return

    @staticmethod
    def colorBox(img, minPix):
        bBox = np.zeros((1,4))
        detect = False
        xyArray = np.transpose(np.nonzero(img))
        shape = xyArray.shape
        if shape[0] >= minPix:
            detect = True
            bBox = cv2.boundingRect(xyArray)
        return bBox, detect

    #ROI SEARCH
    #-------------#
    
    #Decision Tree:
    #Case 0: If Robot has an ROI assigned to it.  
    
    @staticmethod
    def ROIsearch(objList, maskList):
        
        address = 0       
        for robot in objList:
            
            if not robot.ROI == None: 
                cam = robot.cam
                color = robot.color
                roiVals = robot.ROI
                blob = robot.blob                
                ident = (cam, color)
                address = maskList.index(ident)
                
                if (type(address) == int):
                     ROI = maskList[address + 1][roiVals[0]:roiVals[1],roiVals[2]:roiVals[3]]
                     obj = blob.detect(ROI)
                     bBox, colorMatch = Robot.colorBox(ROI, 40)

                     if not obj == []:
                         robot.correctLostNum()
                         #print 'robot', robot.ident, 'found in ROI'
                                                 
                         x = int(obj[0].pt[0] + roiVals[2])
                         y = int(obj[0].pt[1] + roiVals[0])
                         r = int(obj[0].size/2)

                         if r < 25:
                             r += 25
                        
                         robot.selfUpdate(x,y,r)
                         maskList[address + 1] = cv2.circle(maskList[address + 1], (x,y), r, (0,0,0), -1)
                         
                     else:
                         #print 'robot', robot.ident, 'not in ROI'
                         lost = robot.incLostNum(5)
                         
                         if lost == True:
                             #print 'lost'
                             robot.wipeRobot()
                         else:
                             #print 'missing for', robot.lostNum, 'frames'
                             robot.selfUpdate(0,0,0)
        
        return maskList

    
    def getNewCoords(self, scalar):
        vectors = self.getVectors()
        vectors = vectors * scalar
        coords = self.getVectCoords()
        shape = coords.shape
        columns = shape[1]
        newCoords = [0,0,0,0]
        for i in range(columns):
            newCoords[i] = int(vectors[0,i]) + coords[0,i]
        return newCoords
    
    def getBBOX(self):
        scalar = self.getScalar()
        oldCoords = self.getVectCoords()
        newCoords = self.getNewCoords(scalar)
        listCoords = np.vstack((oldCoords[0], newCoords))
        pointset = Utilities.create_PointSet(listCoords)
        BBOX = cv2.boundingRect(pointset)
        return BBOX

#_______________________________________________________#
    #VECTOR METHODS
#---------------------------------------------------#
    def createVect(self, newX, newY):
        oldX, oldY = self.coords
        vx = newX - oldX
        vy = newY - oldY
        self.vector = (vx, vy)
        return vx, vy

    def createVectROI(self):
        values = [1, 1.5, 3.0, 4.5, 6.0]
        (x,y) = self.coords
        (vx, vy) = self.vector
        n = self.lostNum
        roiArray = self.ROI
        r = self.radius
        
        scalar = values[n]
        svx = vx * scalar
        svy = vy * scalar
        newx = x + svx
        newy = y + svy
        points = np.array([[roiArray[3], roiArray[1]],[roiArray[2],roiArray[0]],[newx - r, newy - r],[newx + r, newy + r]])
        bBox = cv2.boundingRect(points)
        roiYA = bBox[1]
        roiXB = bBox[2] + roiXA
        roiYB = bBox[3] + roiYA
        self.ROI = [roiYA, roiYB, roiXA, roiXB]
        return
        
#_______________________________________________________#
    #UPDATE
#---------------------------------------------------#

    #SELF UPDATE
    #--------------#
    #Decision Tree:
    
    #Case 0: If robot was not missing from previous frames:
    #   Create a new Vector and ROI from new coordinates and radius
    
    #Case 1: If robot was missing from previous frames and has a vector:
    #   Use old data to extend ROI to include more area in direction of vector
    
    #Case 2: If robot was was missing from previous frames and vector was not able to be established:
    #   Wipe all data from robot so that it can be searched for 

    def selfUpdate(self, x, y, r):
        
        if self.lostNum == 0:
            self.createVect(x,y)
            self.coords = (x,y)
            self.radius = r            
            
            
        elif ((self.lostNum != 0) and (self.vector != (0,0))):
            oldROI = self.ROI
            oldYa = oldROI[0]
            oldYb = oldROI[1]
            oldXa = oldROI[2]
            oldXb = oldROI[3]
            vX,vY = self.vector
            scalar = self.lostNumAsScalar()
            scvX = vX * scalar
            scvY = vY * scalar
            newYa = int(oldYa + scvY)
            newYb = int(oldYb + scvY)
            newXa = int(oldXa + scvX)
            newXb = int(oldXb + scvX)
            pointList = np.array([[oldXa, oldYa],[oldXb, oldYb],[newXa, newYa],[newXb, newYb]])
            bx, by, bw, bh = cv2.boundingRect(pointList)
            self.ROI = np.array([by, by+bh, bx, bx+bw])

        else:
            self.wipeRobot()
            #print "robot", self.ident, "unestablished vector, cleared"
        return

        
        
    #UPDATE ROBOT DATA
    #--------------#
    #Logic Tree
    #Determine unfound robots create list
    #Compare list to list of found blobs
    #Concatenate found blobs to match number of unfound robots.
    #Pass information in to those robot instances

    #objList:
    #list of robots

    #dataList:
    #list of found robot data

    #colList:
    #list of bools representing columns to be updated
      
    @staticmethod
    def listUpdate(objList, blobDataList, colList, maskList, blobList):

        blobDataArray = np.array(blobDataList)
        blobDataShape = blobDataArray.shape
        unfoundList = Robot.listUnfound(objList)
        maxLen = len(unfoundList)
        dr = 0

        #Empty Condition
        if blobDataList == []:
            return

        #Trim False Positives
        if blobDataShape[0] > maxLen:
            trimList = range(maxLen, blobDataShape[0])
            #print "Num Unfound", maxLen, "Trimming", trimList, "of", blobDataShape[0]
            blobDataArray = np.delete(blobDataArray, trimList, 0)
            blobDataShape = blobDataArray.shape

        rows = []
        for r in range(blobDataShape[0]):
            rows.append(unfoundList[r])
            #print "Appending to robot" ,unfoundList[r]

        for r in rows:

            #Assign Cam
            if colList[0] == True:
                objList[r].cam = blobDataArray[dr,0]
            
            #Assign Coords
            if colList[1] == True:
                objList[r].coords = (blobDataArray[dr,1], blobDataArray[dr,2])

            #Assign Radius
            if colList[2] == True:
                objList[r].radius = blobDataArray[dr,3]
                objList[r].createROI(blobDataArray[dr,1], blobDataArray[dr,2], blobDataArray[dr,3])

            #Assign Vector
            if colList[3] == True:
                objList[r].vector = (blobDataArray[dr,4],blobDataArray[dr,5])

            #Assign Color
            if colList[4] == True:
                objList[r].color = blobDataArray[dr,6]
                objList[r].updateColorProps(maskList, blobList)
                
            dr += 1
        return

#_______________________________________________________#
    #User Interface
#---------------------------------------------------#


    @staticmethod
    def circleFound(image, objList):
        for robot in objList:
            x,y = robot.coords
            r = robot.radius
            
            if not r == 0:
                image = cv2.circle(image, (x,y), r, (0,0,0), 1)
                image = cv2.putText(image, ("%s"% robot.ident), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
        return image


