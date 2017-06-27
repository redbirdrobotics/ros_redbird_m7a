import cv2
import numpy as np
from redbird import Camera

cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 45))
ogreenMask = np.array([[55,45,60],[80,230,230]])
greenMask = np.array([[45,63,32],[100,185,181]])
redMask = np.array([[163,182,136],[180,255,255]])
#loRedMask = np.array([[0,190,190],[3,230,230]])
#upRedMask = np.array([[160,100,100],[180,255,255]])
    

def createMask(img, minThresh, maxThresh, minPixel):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, minThresh, maxThresh)
    count = cv2.countNonZero(mask)

    if (count > minPixel):
        ret = True
    else:
        ret = False
    return ret, mask

def cleanMask(img, minSum):
    shape = img.shape
    for i in range(shape[0]):
        if (np.sum(img[i]) < minSum):
            img[i,:] = 0
    for i in range(shape[1]):
        if (np.sum(img[:,i]) < minSum):
            img[:,i] = 0
    return img

def contourSearch(img, minContours):
    detect = 0
    
    contourDataList = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contourGroupList = contourDataList[1]

    if contourGroupList == []:
        print "is empty"
        return None, False

    else:
        contourList = contourGroupList[0]
        peri = cv2.arcLength(contourList, True)
        approx = cv2.approxPolyDP(contourList, 0.04*peri, True)
        numSides = len(approx)

        area = cv2.contourArea(contourList)
        print dataCollect(numSides, peri, area)
        
        if numSides >= minContours:
            detect = 1

    return contourList, detect, dataArray
        
def dispAvg(tf, boolList):
    boolList.append(tf)
    tot = float(len(boolList))
    true = sum(boolList)
    avg = true/tot
    
    return avg, boolList

def dataCollect(sides, perimeter, area):
    detect = False
    if (sides >= 4 and perimeter >= 100 and area >= 500):
        detect = True
    dataArray = np.array([[sides], [perimeter], [area], [detect]])
    return dataArray

avgList = []

tabulaRasa = np.zeros((720, 1280, 3),np.uint8)

while True:
    
    frame = cam0.getFrame()
    dataArray = np.zeros((3,1))
    
    ret, mask = createMask(frame, greenMask[0], greenMask[1], 100)
    clnMask = cleanMask(mask, 1000)

    c, detect, dataArray = contourSearch(clnMask, 4)
    bbox = cv2.boundingRect(c)
    print c, bbox
    
    if not c == None:
        #avg, avgList = dispAvg(detect, avgList)
        #print "avg", avg
        cv2.drawContours(tabulaRasa, c, -1, (255,255,255), 0)


    esc = Camera.showFrame(tabulaRasa, "mask1")
    if esc == True:
       break
    
cam0.detach()
