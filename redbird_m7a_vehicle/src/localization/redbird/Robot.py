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
        self.found = False
        self.cam = None
        self.camProps = (0,0)
        self.coords = (0,0)
        self.radius = 0
        self.ROI = None
        self.vector = (0,0)
        self.color = 0
        self.blob = None
        self.lostNum = 0
        self.mcoords = (0,0)
        self.height = 0
        self.testLine = None
        self.allData = [self.ident, self.cam, self.coords, self.radius, self.ROI, self.vector, self.color, self.lostNum, self.mcoords]
        return

#_______________________________________________________#
    #GENERAL
#---------------------------------------------------#

    def wipeRobot(self):
        self.found = False
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
    def listUnfound(objList, foundList):

        listUnfound = range(len(objList))
        for num in range(len(foundList)):
            listUnfound.remove(foundList[num])
                   
        return listUnfound

    @staticmethod
    def printAllData(objList):
        pass
        # for robot in objList:
        #     print robot.allData
        # return

#_______________________________________________________#
    #COORDINATE CONVERSION
#---------------------------------------------------#

    def coordConvert(self, xN, yN, xAxis, yAxis, height):
        # print 'Robot', self.ident, 'is at', self.coords
        x, y = self.coords
        theta = xAxis[x]
        phi = yAxis[y]
        yDist = height*np.tan(phi)
        xDist = yDist*np.tan(theta)
        self.mcoords = (xDist + xN, yDist + yN)
        return
    
    @staticmethod
    def listConversion(objQuad, objRobotList, objCamList):
        xN, yN = objQuad.mcoords
        h = objQuad.height

        foundList = [index for index, robot in enumerate(objRobotList) if robot.found == True]

        for robot in foundList:
            xAxis = objCamList[objRobotList[robot].cam].xAxis
            yAxis = objCamList[objRobotList[robot].cam].yAxis
            objRobotList[robot].coordConvert(xN, yN, xAxis, yAxis, h)
        return foundList
#_______________________________________________________#
    #LOST NUMBER METHODS
#---------------------------------------------------#

    def incLostNum(self, threshold):
        lost = False
        self.lostNum = self.lostNum + 1
        if self.lostNum == threshold:
            lost = True
            self.ROI = None
            self.lostNum = 0
        return lost

    def lostNumAsScalar(self):
        scalarVals = [1, 3, 4, 5.5, 7, 8.5]
        scalar = scalarVals[self.lostNum]
        return scalar
    
#_______________________________________________________#
    #COLOR METHODS
#---------------------------------------------------#
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
    

    @staticmethod
    def checkDist(objList, dataArray, buff, foundList):
        if foundList == []:
            return dataArray
        
        #print dataArray.shape, "Robots Found"
        #print len(foundList), 'Robots exist'

        rFound = len(foundList)
        dataShape = dataArray.shape
        iR = 0
        iD = 0

        while (iR < rFound) and (iD < dataShape[0]):

            rX, rY = objList[(foundList[iR])].coords
            bX, bY, bR = dataArray[iD,1:4]

            if abs(rX - bX) <= buff or abs(rY - bY) <= buff:
                #print "Appending to Robot", objList[foundList[iR]].ident
                objList[foundList[iR]].lostNum = 0
                objList[foundList[iR]].selfUpdate(bX, bY, bR, objList[iR].camProps)
                dataArray = np.delete(dataArray, iD, 0)
                dataShape = dataArray.shape
                iR += 1
                iD = 0

            else:
                #print dataArray[iD,1:4], 'does not belong to Robot', objList[foundList[iR]].ident
                iD += 1
                
                if (iD == dataShape[0]) and (iR < rFound):
                    iR += 1
                    iD = 0
                    #print 'now checking robot', objList[foundList[iR]].ident
                    
            #print 'loop vairables', iR, '<=', rFound, 'and', iD, '<', dataShape[0]
                
        return dataArray

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
    def ROIsearch(objList, maskList, foundList):

        address = 0

        if foundList == []:
            return maskList, foundList
        
        for rNum in foundList:

            roiVals = objList[rNum].ROI
            blob = objList[rNum].blob
            
            address = maskList.index((objList[rNum].cam, objList[rNum].color))
            
            if not type(address) == int:
                #print 'incorrect address'
                return
            
            ROI = maskList[address + 1][roiVals[0]:roiVals[1],roiVals[2]:roiVals[3]]
            obj = blob.detect(ROI)

            if not obj == []:
                
                objList[rNum].lostNum = 0
                #print "robot", objList[rNum].ident, "found in ROI"

                x = int(obj[0].pt[0] + roiVals[2])
                y = int(obj[0].pt[1] + roiVals[0])
                r = int(obj[0].size/2)
                #print 'Robot', objList[rNum].ident, 'radius', r

                if r < 35:
                    r += 35

                objList[rNum].selfUpdate(x, y, r, objList[rNum].camProps)
                maskList[address + 1] = cv2.circle(maskList[address + 1], (x,y), r, (0,0,0), -1)
                
            else:
                #print 'robot', objList[rNum].ident, 'not in ROI, last seen at', objList[rNum].coords
                
##                #FOR TROUBLESHOOTING ONLY
##                cv2.imshow("ROI", ROI)
##                k = cv2.waitKey(30) & 0xff
##                esc = False
##                if k == 27:
##                    esc = True
##                    break
##                #_________________________#
                
                lost = objList[rNum].incLostNum(5)
                         
                if lost == True:
                    #print 'lost'
                    objList[rNum].wipeRobot()
                    foundList.remove(rNum)
                else:
                    #print 'missing for', objList[rNum].lostNum, 'frames'
                    objList[rNum].selfUpdate(0, 0, 0, objList[rNum].camProps)
        
        return maskList, foundList

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

    def quadUpdate(self, x, y, h):
        self.mcoords = (x, y)
        self.height = h
        return
    
    #SELF UPDATE
    #--------------#
    #Decision Tree:
    
    #Case 0: If robot was not missing from previous frames:
    #   Create a new Vector and ROI from new coordinates and radius
    
    #Case 1: If robot was missing from previous frames and has a vector:
    #   Use old data to extend ROI to include more area in direction of vector
    
    #Case 2: If robot was was missing from previous frames and vector was not able to be established:
    #   Expand ROI in all directions

    def selfUpdate(self, x, y, r, (hRes, vRes)):
        
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
            newYa = min(abs(int(oldYa + scvY)), vRes)
            newYb = min(abs(int(oldYb + scvY)), vRes)
            newXa = min(abs(int(oldXa + scvX)), hRes)
            newXb = min(abs(int(oldXb + scvX)), hRes)
            pointList = np.array([[oldXa+r, oldYa],[oldXb+r, oldYb],[newXa, newYa],[newXb, newYb]])
            bx, by, bw, bh = cv2.boundingRect(pointList)
            self.ROI = np.array([by, by+bh, bx, bx+bw])
            
        else:
            oldROI = self.ROI
            oldYa = oldROI[0]
            oldYb = oldROI[1]
            oldXa = oldROI[2]
            oldXb = oldROI[3]
            vX, vY = self.vector
            scalar = self.lostNumAsScalar()
            scvX = vX * scalar
            scvY = vY * scalar
            self.ROI = np.array([int(oldYa - scvY), int(oldYb + scvY), int(oldXa - scvX), int(oldXb + scvX)])
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
    def listUpdate(objList, blobDataList, colList, maskList, blobList, foundList, camList):

        #Empty Condition
        if blobDataList == []:
            return
        
        blobDataArray = np.array(blobDataList)

        #Check if Found Coords Corespond to a Lost Robot
        newBlobDataArray = Robot.checkDist(objList, blobDataArray, 30, foundList)

        #Empty Condition
        if newBlobDataArray == []:
            return

        #print 'postfunction dataArray', newBlobDataArray
        blobDataShape = newBlobDataArray.shape
        
        unfoundList = Robot.listUnfound(objList, foundList)
        maxLen = len(unfoundList)
        dr = 0
        
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
            
            objList[r].found = True
            
            #Assign Cam
            if colList[0] == True:
                objList[r].cam = blobDataArray[dr,0]
                objList[r].camProps = (camList[objList[r].cam].hRes, camList[objList[r].cam].vRes)
            
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
            x, y = robot.coords
            vX, vY = robot.vector
            r = robot.radius
            mx, my = robot.mcoords
            mx = round(mx, 2)
            my = round(my, 2)
            
            if not r == 0:
                image = cv2.circle(image, (x,y), r, (0,0,0), 1)
                image = cv2.putText(image, ("%s"% robot.ident), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
                #image = cv2.putText(image, ("mx: %s Vx %s" % (mx, vX)), (x-10, y+15), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
                #image = cv2.putText(image, ("my: %s Vy %s" % (my, vY)), (x-10, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
                #image = cv2.line(image, (x,y), (x + vX, y + vY), (0,0,0), 5)
        return image


