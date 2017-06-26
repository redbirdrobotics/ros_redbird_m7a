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

    def checkLostNum(self, threshold):
        if self.lostNum == threshold:
            print "kicked"
            self.ROI = None
            self.lostNum = 0
        return
    
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
    def getROI(self):
        return self.ROI

    @staticmethod
    def listROI(objList):
        length = len(objList)
        listROI = [0] * length
        
        for i in range(length):
            listROI[i] = objList[i].ROI
        return listROI

##    @staticmethod
##    def blackCircle(roboList, maskList):
##        length = len(maskList)
##        for robot in robotList:
##            x,y = robot.coords
##            r = robot.radius
##            if not r == 0:
##                for m in range(1, maskList, 2):
##                    mask = 
##                    mask = cv2.circle(mask, (x,y), r+25, (0,0,0), -1)
##        return

    def xyrc2ROI(self):
        x,y = self.coords
        r = self.radius
        self.ROI = np.array([y-r, y+r, x-r, x+r])

    def setCoords(self, new_coords):        
        old_coords = np.asarray(self.coords)
        numRows = old_coords.shape
        row = (numRows[0] - 1)
        #if old coords and new coords are not equal
        if not (np.array_equal(old_coords[row,:], new_coords)):
            self.coords = np.vstack((old_coords, new_coords))
        return

    def setROI(self, image, coords, pad):
        if not np.all(coords == 0):
            shape = self.coords.shape
            history = shape[0]
            if self.lostNum > 0:
                x,y,w,h = self.getBBOX()
                self.ROI = image[y:y+h, x:x+w]
            elif shape[0] > 1:
                x = int(coords[0])
                y = int(coords[1])
                w = int(coords[2])
                h = int(coords[3])
                self.ROI = image[y:y+h+pad, x:x+w+pad]
        return
    @staticmethod
    def contourSearch(ROI):
        c = cv2.findContours(ROI, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        per = cv2.arcLength(c[0], True)
        approx = cv2.approxPolyDP(c, 0.04*per, True)
        print len(approx)
        return
    
    @staticmethod
    def ROIsearch(objList, maskList):
        
        address = 0       
        for robot in objList:
            
            if not robot.ROI == None:
                print "ROI not None"
                cam = robot.cam
                color = robot.color
                roiVals = robot.ROI
                blob = robot.blob                
                ident = (cam, color)
                address = maskList.index(ident)
                print "preloop address", address
                #mask = maskList[address + 1]
                
                if (type(address) == int):
                     print "address", address
                     ROI = maskList[address + 1][roiVals[0]:roiVals[1],roiVals[2]:roiVals[3]]
                     Robot.contourSearch(ROI)
                     obj = blob.detect(ROI)

                     if not obj == []:
                         print 'found in ROI'
                                                 
                         x = int(obj[0].pt[0] + roiVals[2])
                         y = int(obj[0].pt[1] + roiVals[0])
                         r = int(obj[0].size/2)

                         if r < 25:
                             r += 25
                        
                         robot.coords = (x,y)
                         robot.radius = r
                         maskList[address + 1] = cv2.circle(maskList[address + 1], (x,y), r+25, (0,0,0), -1)
                         
                     else:
                         print 'not in ROI'
                         lost = robot.incLostNum(15)

                         if lost == True:
                             print 'lost'
                             robot.coords = (0,0)
                             robot.radius = 0
                             robot.ROI = None
            
        return maskList


    def getScalar(self):
        num = self.lostNum
        values = np.array([1.5, 3.0, 4.5, 6.0])
        return values[num]
    
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
    #Returns coordinates needed to create vector
    def getVectCoords(self):
        #get object coords and format as np array
        coords = np.asarray(self.coords)
        #find how many rows long
        shape = coords.shape
        row = (shape[0] - 1)
        #get most recent set of coords
        set1 = coords[row,]
        #search backwards to find coords that are not equal to most recent or zero
        set2 = Utilities.find_nEqual_nZero(set1, coords, False)
        #stack them
        coordsArray = np.vstack((set1, set2))
        return coordsArray

    #Creates a vector from objects coords
    def getVectors(self):
        #get needed coords
        vectorArray = self.getVectCoords()
        #find how many columns long
        shape = vectorArray.shape
        column = shape[1]
        #create array of correct size
        vector = np.zeros((1,4), dtype=int)
        #subtract past coords from most recent 
        for i in range(column):
            vector[0,i] = vectorArray[0,i] - vectorArray[1,i]
        return vector
        
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
    def update(mode, objList, dataList, colList, maskList, blobList):
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

            #Upadte Cam
            if colList[0] == True:
                objList[r].cam = dataArray[dr,0]
            
            #Update Coords
            if colList[1] == True:
                objList[r].coords = (dataArray[dr,1], dataArray[dr,2])

            #Update Radius
            if colList[2] == True:
                objList[r].radius = dataArray[dr,3]
                objList[r].xyrc2ROI()

            #Update Vector
            if colList[3] == True:
                objList[r].vector = (dataArray[dr,4],dataArray[dr,5])

            #Update Color
            if colList[4] == True:
                objList[r].color = dataArray[dr,6]
                objList[r].updateColorProps(maskList, blobList)
                
            dr += 1
        return

#_______________________________________________________#
    #Testing In Main
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




