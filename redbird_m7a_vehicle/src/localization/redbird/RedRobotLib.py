import rospy
import cv2
import numpy as np

class RedRobot():

    def __init__(self, num):
        self.ident = num
        self.color = 0
        self.found = False
        self.cam = None
        self.camProps = (0,0)
        self.camAxis = (0,0)
        self.coords = (0,0)
        self.radius = 0
        self.ROI = (0,0,0,0)
        self.vector = (0,0)
        self.lostNum = 0
        self.mcoords = (0,0)
        return

#_______________________________________________________#
    #GENERAL
#---------------------------------------------------#

    def wipeRobot(self):
        self.found = False
        self.cam = None
        self.camProps = (0,0)
        self.coords = (0,0)
        self.radius = 0
        self.ROI = (0,0,0,0)
        self.vector = (0,0)
        self.lostNum = 0
        self.mcoords = (0,0)
        return

    @staticmethod
    def sortFound(objList, foundList, unfoundList):
        if not foundList and not unfoundList:
            for robot in objList:
                if robot.found == True:
                    foundList.append(robot)
                else:
                    unfoundList.append(robot)
        else:
            iF = 0
            for robot in foundList:
                if robot.found == False:
                    unfoundList.append(foundList.pop(iF))
                    iF += 1
                else:
                    iF += 1

            iU = 0
            for robot in unfoundList:
                if robot.found == True:
                    foundList.append(unfoundList.pop(iU))
                    iU += 1
                else:
                    iU += 1
        return

    @staticmethod
    def listFound(objList):
        pass
        # for robot in objList:
        #     #print 'Robot', robot.ident, robot.found
        #     pass
        # return

#_______________________________________________________#
    #COORDINATE CONVERSION
#---------------------------------------------------#

    def cvt2meters(self, xA, yA, height, xAxis, yAxis):
        #print 'Robot', self.ident, 'is at', self.coords
        x, y = self.coords
        theta = xAxis[x]
        phi = yAxis[y]
        yDist = height*np.tan(phi)
        xDist = height*np.tan(theta)
        self.mcoords = (xDist + xA, yDist + yA)
        return

    @staticmethod
    def listcvt2meters(quadDataList, foundList, camList):
        if not foundList:
            return

        for robot in foundList:
            xAxis = camList[robot.cam].xAxis
            xAxisQ = [x + quadDataList[3] for x in xAxis]
            yAxis = camList[robot.cam].yAxis
            yAxisQ = [y + quadDataList[4] for y in yAxis]
            robot.cvt2meters(quadDataList[0], quadDataList[1], quadDataList[2], xAxisQ, yAxisQ)

#_______________________________________________________#
    #LOST NUMBER METHODS
#---------------------------------------------------#

    def incLostNum(self, maxFrames):
        self.lostNum += 1
        if self.lostNum == maxFrames:
            return True
        else:
            return False

    def cvt2scalar(self):
        scalarVals = [1,3,4,5.5,7,8.5]
        return scalarVals[self.lostNum]

#_______________________________________________________#
    #ROI METHODS
#---------------------------------------------------#

    @staticmethod
    def checkDist(foundList, dataList, rad):
        if not foundList:
            return

        maxFound = len(foundList)
        maxData = len(dataList)
        iR = 0
        iD = 0

        while (iR < maxFound) and (iD < maxData):

            rX, rY = foundList[iR].coords
            cam, bX, bY, bR, vX, vY = dataList[iD]

            if abs(rX - bX) <= rad or abs(rY - bY) <= rad:
                foundList[iR].lostNum = 0
                foundList[iR].selfUpdate(bX, bY, bR)
                del dataList[iD]
                maxData = len(dataList)
                iR += 1
                iD = 0

            else:
                iD +=1

                if (iD == maxData) and (iR < maxFound):
                    iR += 1
                    iD = 0
        return

    def createROI(self, x, y, r):
        hRes, vRes = self.camProps

        if (x-r <= 0) or (y-r <= 0):
            self.ROI = (0, y+r, 0, x+r)

        elif (x+r >= hRes) or (y+r >=  vRes):
            self.ROI = (y-r, vRes-1, x-r, hRes-1)

        else:
            self.ROI = (y-r, y+r, x-r, x+r)

        return

    @staticmethod
    def ROIsearch(foundList, imgList, detector):
        address = 0

        if not foundList:
            return

        for robot in foundList:

            rY,rH,rX,rW = robot.ROI
            ROI = imgList[robot.cam][rY:rH,rX:rW]
            keypoints = detector.detect(ROI)

            if keypoints:

                robot.lostNum = 0
                bX = int(keypoints[0].pt[0] + rX)
                bY = int(keypoints[0].pt[1] + rY)
                bR = int(keypoints[0].size/2)
                #print 'Robot', robot.ident, 'found in ROI', bX, bY

                if bR < 35:
                    bR += 35

                robot.selfUpdate(bX, bY, bR)
                imgList[robot.cam] = cv2.circle(imgList[robot.cam], (bX, bY), bR, (0,0,0), -1)

            else:
                #print 'Robot', robot.ident, 'not found in ROI'
                lost = robot.incLostNum(5)
                if lost == True:
                    #print 'lost'
                    robot.wipeRobot()
                else:
                    #print 'missing for', robot.lostNum
                    robot.selfUpdate(0,0,0)
        return

#_______________________________________________________#
    #VECTOR METHODS
#---------------------------------------------------#

    def createVect(self, newX, newY):
        oldX, oldY = self.coords
        vX = newX - oldX
        vY = newY - oldY
        self.vector = (vX, vY)
        return

#_______________________________________________________#
    #UPDATE
#---------------------------------------------------#

    #SELF UPDATE
    #--------------#
    #Logic:
    #Case 0: If robot was not missing from previous frames:
    #   Create a new Vector and ROI from new coordinates and radius

    #Case 1: If robot was missing from previous frames and has a vector:
    #   Use old data to extend ROI to include more area in direction of vector

    #Case 2: If robot was was missing from previous frames and vector was not able to be established:
    #   Expand ROI in all directions

    #Arguments:
    #x & y: coordinates of found blob
    #r: Radius of found blob

    def selfUpdate(self, x, y, r):

        if self.lostNum == 0:
            self.createROI(x, y, r)
            self.createVect(x,y)
            self.coords = (x,y)
            self.radius = r

        elif ((self.lostNum != 0) and (self.vector != (0,0))):
            hRes, vRes = self.camProps
            rY, rH, rX, rW = self.ROI
            vX, vY = self.vector
            scalar = self.cvt2scalar()
            scvX = vX*scalar
            scvY = vY*scalar
            newY = min(abs(int(rY + scvY)), vRes)
            newH = min(abs(int(rH + scvY)), vRes)
            newX = min(abs(int(rX + scvX)), hRes)
            newW = min(abs(int(rW + scvX)), hRes)
            pointList = np.array([[rX, rY],[rW, rH],[newX, newY],[newW, newH]])
            bX, bY, bW, bH = cv2.boundingRect(pointList)
            #print 'boundingrect', bX, bY, bW, bH
            self.ROI = (bY, bY+bH, bX, bX+bW)

        else:
            hRes, vRes = self.camProps
            rY, rH, rX, rW = self.ROI
            newY = max(rY - 30, 0)
            newH = min(rH + 30, vRes)
            newX = max(rX - 30, 0)
            newW = min(rW + 30, hRes)
            self.ROI = (newY, newH, newX, newW)
        return


    #UPDATE ROBOT DATA
    #--------------#
    #Logic:
    #Check to see if any coordinates in dataList are within radius of a missing robot
    #Concatenate found blobs to match number of unfound robots.
    #Pass information in to those robot instances

    #Arguments:
    #foundList: list of found robot instances

    #unfoundList: list of unfound instances

    #dataList: list of found blob data

    #camList: list of camera instances

    @staticmethod
    def listUpdate(foundList, unfoundList, dataList, camList):

        if not dataList or not unfoundList:
            return

        RedRobot.checkDist(foundList, dataList, 30)

        if not dataList:
            return

        maxData = len(dataList)
        maxUnfound = len(unfoundList)

        if maxData > maxUnfound:
            del dataList[maxUnfound:maxData]
            maxData = len(dataList)

        for i in range(maxData):

            cam, x, y, r, vX, vY = dataList[i]
            #print 'Updating unfoundRobot', i, 'With', x,y

            #Update Found Status
            unfoundList[i].found = True

            #Assign Cam
            unfoundList[i].cam = cam
            unfoundList[i].camProps = (camList[cam].hRes, camList[cam].vRes)

            #Assign Coords
            unfoundList[i].coords = (x,y)

            #Assign Radius
            unfoundList[i].radius = r
            unfoundList[i].createROI(x, y, r)

            #Assign Vector
            unfoundList[i].vector = (vX, vY)
        return
