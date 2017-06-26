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
        scalarVals = [1, 1.5, 2, 3.5, 4, 4.5]
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


    @staticmethod
    def ROIsearch(objList, maskList):
        
        address = 0       
        for robot in objList:
            
            if not robot.ROI == None:
                print "ROI", robot.ROI 
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
                         print 'found in ROI'
                                                 
                         x = int(obj[0].pt[0] + roiVals[2])
                         y = int(obj[0].pt[1] + roiVals[0])
                         r = int(obj[0].size/2)

                         if r < 25:
                             r += 25
                        
                         robot.selfUpdate(x,y,r)
                         maskList[address + 1] = cv2.circle(maskList[address + 1], (x,y), r, (0,0,0), -1)
                         
                     else:
                         print 'not in ROI'
                         lost = robot.incLostNum(5)
                         
                         if lost == True:
                             print 'lost'
                             robot.wipeRobot()
                         else:
                             print 'missing for', robot.lostNum, 'frames'
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
        print "coords", (x,y)
        (vx, vy) = self.vector
        n = self.lostNum
        roiArray = self.ROI
        print "roi", roiArray
        r = self.radius
        
        scalar = values[n]
        svx = vx * scalar
        svy = vy * scalar
        newx = x + svx
        newy = y + svy
        points = np.array([[roiArray[3], roiArray[1]],[roiArray[2],roiArray[0]],[newx - r, newy - r],[newx + r, newy + r]])
        print points
        #Work on this
        bBox = cv2.boundingRect(points)
        roiYA = bBox[1]
        roiXB = bBox[2] + roiXA
        roiYB = bBox[3] + roiYA
        self.ROI = [roiYA, roiYB, roiXA, roiXB]
        return
        
#_______________________________________________________#
    #UPDATE
#---------------------------------------------------#

    @staticmethod
    def listUnfound(objList):
        ret = False
        listUnfound = []
        lgthList = len(objList)
        
        for r in range(lgthList):
            if all(v == 0 for v in objList[r].coords):
                listUnfound.append(r)

        lgthUnfound = len(listUnfound)
        if lgthUnfound == lgthList:
            ret = True
            
        return listUnfound, ret

    #SELF UPDATE
    #--------------#
    #Logic Tree:
    
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
            newYa = oldYa + scvY
            newYb = oldYb + scvY
            newXa = oldXa + scvX
            newXb = oldXb + scvX
            pointList = np.array([[oldXa, oldYa],[oldXb, oldYb],[newXa,newYa],[newXb,newYb]])
            bx, by, bw, bh = cv2.boundingRect(pointList)
            self.ROI = np.array([by, by+bh, bx, bx+bw])

        else:
            self.wipeRobot()
        return

        
        
    #UPDATE ROBOT DATA
    #--------------#
    #mode:
    #Case 0: Update only Robots that are unfound
    #Case 1: Array is Formatted to Update Specific Robots, ie. row 0 = robot 0

    #objList:
    #list of robots

    #dataList:
    #list of found robot data

    #colList:
    #list of bools representing columns to be updated
      
    @staticmethod
    def listUpdate(mode, objList, dataList, colList, maskList, blobList):
        dataArray = np.array(dataList)
        shape = dataArray.shape
        dr = 0

        if dataList == []:
            return
        
        if mode == 0:        
            unfound, ret = Robot.listUnfound(objList)
            lgth = len(unfound)

            #All Robots are unfound
            if ret == True:
                print "ret pass"
                rows = range(shape[0])

            #Fill in gaps
            else:
                print "else pass", unfound
                rows = []
                for r in range(shape[0]):
                    rows.append(unfound[r])

        elif mode == 1:
            rows = range(shape[0])

        for r in rows:

            #Assign Cam
            if colList[0] == True:
                objList[r].cam = dataArray[dr,0]
            
            #Assign Coords
            if colList[1] == True:
                objList[r].coords = (dataArray[dr,1], dataArray[dr,2])

            #Assign Radius
            if colList[2] == True:
                objList[r].radius = dataArray[dr,3]
                objList[r].createROI(dataArray[dr,1], dataArray[dr,2], dataArray[dr,3])

            #Assign Vector
            if colList[3] == True:
                objList[r].vector = (dataArray[dr,4],dataArray[dr,5])

            #Assign Color
            if colList[4] == True:
                objList[r].color = dataArray[dr,6]
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
                image = cv2.circle(image, (x,y), r, (0,0,0), -1)
                image = cv2.putText(image, ("%s"% robot.ident), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
        return image

#_______________________________________________________#
    #Testing
#---------------------------------------------------#

##someList = [True, True, False, True]
##for val in someList:
##    print val

##endList = []
##newList = [[1,2,3,4,5]]
##endList.extend(newList)
##newerList = [[2,3,4,5,6]]
##endList.extend(newerList)
##print endList
##endArray = np.asarray(endList)
##print endArray
##
##bill = Robot()
##murray = Robot()
##isvery = Robot()
##awesome = Robot()

##whoyagunnacall = [bill, murray, isvery, awesome]
##
##for robot in whoyagunnacall:
##    print robot.coords

##bill.coords = (9,2)
##
###stayPuft = np.array([[1,23,4,5,6,1],[2,45,67,2,4,1],[0,0,0,0,0,0],[1,1,1,1,1,1]])
##stayPuft = [[1,1,23,4,6,6,0], [1,2,45,676,6,0,1], [0,0,0,0,0,0,0], [0,1,1,1,1,1,1]]
##
##Robot.update(1, whoyagunnacall, stayPuft, [True, True, True, False, True])
##
##print bill.cam, bill.coords, bill.radius, bill.vector, bill.color
##print murray.cam, murray.coords, murray.radius, murray.vector, murray.color
##print isvery.cam, isvery.coords, isvery.radius, isvery.vector, isvery.color
##print awesome.cam, awesome.coords, awesome.vector, awesome.color
##
##print Robot.listUnfound(whoyagunnacall)
##print Robot.frameList2ROIList(whoyagunnacall, frameList)




