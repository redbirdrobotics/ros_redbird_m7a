import cv2
import numpy as np

class RedRobot():

    def __init__(self, num):
        self.ident = num
        self.found = False
        self.cam = None
        self.camAxis = (0,0)
        self.coords = (0,0)
        self.radius = 0
        self.ROI = (0,0,0,0)
        self.vector = (0,0)
        self.lostNum = 0
        self.mcoords = (0,0,)
        return

#_______________________________________________________#
    #GENERAL
#---------------------------------------------------#

    def wipeRobot(self):
        self.ident = num
        self.found = False
        self.cam = None
        self.coords = (0,0)
        self.radius = 0
        self.ROI = (0,0,0,0)
        self.vector = (0,0)
        self.lostNum = 0
        self.mcoords = (0,0,)
        return

    @staticmethod
    def sortFound(objList, foundList, unfoundList):
        for robot in objList:
            if robot.found == True:
                foundList.append(robot)
            else:
                unfoundList.append(robot)
        return
#_______________________________________________________#
    #COORDINATE CONVERSION
#---------------------------------------------------#

    def cvt2meters(self, xA, yA, height, xAxis, yAxis):
        print 'Robot', self.ident, 'is at', self.coords
        x, y = self.coords
        theta = xAxis[x]
        phi = yAxis[y]
        yDist = height*np.tan(phi)
        xDist = height*np.tan(theta)
        self.mcoords = (xDist + xA, yDist + yA)
        return

    @staticmethod
    def listcvt2meters(xA, yA, h, foundList, imgList):
        for robot in foundList:
            xAxis = imgList[robot.cam].xAxis
            yAxis = imgList[robot.cam].yAxis
            robot.cv2meters(xA, yA, h, xAxis, yAxis)
        return

#_______________________________________________________#
    #LOST NUMBER METHODS
#---------------------------------------------------#

    def IncLostNum(self, maxFrames):
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

    def checkDist(foundList, dataList, rad):
        if not foundList:
            return

        maxFound = len(foundList)
        maxData = len(dataList)
        iR = 0
        iD = 0

        while (iR < maxFound) and (iD < maxData[0]):

            rX, rY = foundList.coords
            cam, bX, bY, bR, vX, vY = datalist[iD]

            if abs(rX - bX) <= rad or abs(rY - bY) <= rad:
                foundList.lostNum = 0
                foundList.selfUpdate(bX, bY, bR)
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
        if (x-r == 0) or (y-r == 0):
            self.ROI = (0, y+r, 0, x+r)
        else:
            self.ROI = (y-r, y+r, x-r, x+r)
        return

    @staticmethod
    def ROIsearch(foundList, imgList, detector):
        address = 0

        if not foundList:
            return

        for robot in foundList:

            rX,rY,rW,rH = robot.ROI

            ROI = imgList[robot.cam][y:w,x:h]
            keypoints = detector.detect(ROI)

            if keypoints:

                self.lostNum = 0
                bX = int(keypoints[0].pt[0] + rW)
                bY = int(keypoints[0].pt[1] + rX)
                bR = int(keypoints[0].size/2)

                if bR < 35:
                    bR += 35

                robot.selfUpdate(bX, bY, bR)
                imgList[robot.cam] = cv2.circle(imgList[robot.cam], (bX, bY), bR, (0,0,0), -1)

            else:
                if robot.incLostNum(5):
                    robot.wipeRobot()
                    foundList.remove(robot)
                else:
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

    def selfUpdate(self, x,y,r,):

        if self.lostNum == 0:
            self.createVect(x,y)
            self.coords = (x,y)
            self.radius = r

        elif ((self.lostNum != 0) and (self.vector != (0,0))):
            hRes, vRes = robot.camProps
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
            bX, bY, bW, bH = cv2.boundingRect(pointlist)
            self.ROI = (bY, bY+bH, bX, bX+bW)

        else:
            rY, rH, rX, rW = self.ROI
            newY = min(abs(rY - 30), vRes)
            newH = min(abs(rH + 30), vRes)
            newX = min(abs(rX - 30), hRes)
            newW = min(abs(rW + 30), hRes)
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
    def listupdate(foundList, unfoundList, dataList, camList):

        if not dataList:
            return

        RedRobot.checkDist(foundList, dataList, 30)

        if not dataList:
            return

        maxData = len(dataList)
        maxUnfound = len(unfound)

        if maxData > maxUnfound:
            del dataList[maxUnfound:maxData]
            maxData = len(dataList)

        for i in range(maxData):

            cam, x, y, r, vX, vY = dataList[i]

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

##############################################################
####################### Utilities Class #########################
##############################################################
    
class Utilities():

#_______________________________________________________#
    #Mask
#---------------------------------------------------#

    @staticmethod
    def createHSVMask(img, minThresh, maxThresh):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, minThresh, maxThresh)
        count = cv2.countNonZero(mask)
        return mask
        
    @staticmethod
    def getMaskList(frameList, maskValsList, maskList):
        for frame in frameList:
            hsvMask = Utilities.createHSVMask(frame, maskVal[0], maskVal[1])
            maskList.append(hsvMask)
        return

#_______________________________________________________#
    #User Interface
#---------------------------------------------------#

    @staticmethod
    def circleFound(image, foundList):
        for robot in foundList:
            x, y = robot.coords
            vX, vY = robot.vector
            r = robot.radius
            mx, my = robot.mcoords
            mx = round(mx, 2)
            my = round(my, 2)
            
            image = cv2.circle(image, (x,y), r, (0,0,0), 1)
            image = cv2.putText(image, ("%s"% robot.ident), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
            #image = cv2.putText(image, ("mx: %s Vx %s" % (mx, vX)), (x-10, y+15), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
            #image = cv2.putText(image, ("my: %s Vy %s" % (my, vY)), (x-10, y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 255, 2)
            #image = cv2.line(image, (x,y), (x + vX, y + vY), (0,0,0), 5)
        return image    

#_______________________________________________________#
    #Detector
#---------------------------------------------------#

    @staticmethod
    def getParams(object, detectorType):

        #Ground Robot Parameters
        if detectorType == 0:
            
            #Thresholds
            object.minThreshold = 0
            object.maxThreshold = 256

            #Filter by Color
            object.filterByColor = True
            object.blobColor = 255

            #Filter by Area
            object.filterByArea = True
            object.minArea = 50
            object.maxArea = 6000

            #Filter by Circularity
            object.filterByCircularity = False
            object.minCircularity = 0.1

            #Filter by Convexity
            object.filterByConvexity = False
            object.minConvexity = 0.5

            #Filter by Inertia
            object.filterByInertia = False
        return

    @staticmethod
    def blobSearch(maskList, detector, dataList):
        cam = 0
        detect = True
        
        while detect == True:
            keypoints = detector.detect(maskList[cam])

            if not keypoints:
                cam +=1
                detect = False
                
                if cam > (len(camList) - 1):
                    return

            else:
                x = int(keypoints[0].pt[0])
                y = int(keypoints[0].pt[1])
                r = int(keypoints[0].size/2)

                if r<35:
                    r+= 35

                dataList.append([cam,x,y,r,0,0])
                maskList[cam] = cv2.circle(maskList[cam], (x,y), r, (0,0,0), -1)
        return

##############################################################
####################### Camera Class  #########################
##############################################################

class Camera():

    #PORT: USB PORT CAMERA IS ATTACHED TO
    #hRes, vRes: HORIZONTAL AND VERTICAL RESOLUTION
    #FPS: FRAMES PER SECOND
    #hRange, vRange: HORIZONTAL AND VERTICAL RANGE OF LENS IN DEGREES
    #azimuth: AZIMUTHAL ANGLE IN DEGREES, 0 IS FACING FORWARD RELATIVE TO DRONE ORIENTATION PROGRESSING CLOCKWISE
    #altitude: ALTITUDINAL ANGLE IN DEGREE, 0 IS FACING DOWNWARD RELATIVE TO DRONE ORIENTATION PROGRESSING UP
    def __init__(self, port, (hRes, vRes), FPS, (hRange, vRange), (azimuth, altitude)):
        self.port = port
        self.feed = cv2.VideoCapture(port)
        ret, self.frame = self.feed.read()
        self.hRes = self.feed.set(cv2.CAP_PROP_FRAME_WIDTH, hRes)
        self.vRes = self.feed.set(cv2.CAP_PROP_FRAME_HEIGHT, vRes)
        self.setFPS = self.feed.set(cv2.CAP_PROP_FPS, FPS)
        self.lensRange = np.radians(hRange), np.radians(vRange)
        self.orientation = np.radians(azimuth), np.radians(altitude)
        self.xAxis = np.zeros((1, hRes))
        self.yAxis = np.zeros((1, vRes))
        return

    def detach(self):
        self.feed.release()
        return

    def getFrame(self):
        ret, frame = self.feed.read()       
        return frame
    
    @staticmethod
    def getFrameList(camList):
        frameList = []
        length = len(camList)
        for c in range(length):
            ret, frame = camList[c].feed.read()
            frameList.append(frame)
        return frameList
    
    @staticmethod
    def showFrame(frame, str):
        cv2.imshow(str, frame)
        k = cv2.waitKey(30) & 0xff
        esc = False
        if k == 27:
            esc = True
        return esc
        
    def getRes(self):
        hRes = int(self.feed.get(cv2.CAP_PROP_FRAME_WIDTH))
        vRes = int(self.feed.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return hRes, vRes
    
#_______________________________________________________#
    #Pixel Mapping 
#---------------------------------------------------#

    def createAxis(self):
        hRes, vRes = self.getRes()
        hRange, vRange = self.lenRange
        hResMid = int(hRes/2)
        vResMid = int(vRes/2)
        hRangeMid = int(hRange/2)
        vRangeMid = int(vRange/2)
        az, al = self.orientation

        #Create X & Y Axis of Angle Map
        
        #X AXIS
        self.xAxis = np.linspace((-hRange_Mid + az), (hRange_Mid + az), hRes)
        
        #Y AXIS
        self.yAxis = np.linspace((vRange_Mid + al), (-vRange_Mid + al), vRes)
        return        
        

    
    
