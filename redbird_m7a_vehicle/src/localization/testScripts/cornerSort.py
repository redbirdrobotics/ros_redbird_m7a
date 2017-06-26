import cv2
import numpy as np 
import time
from redbird import Camera, Utilities
import os 
import math

np.set_printoptions(threshold=np.nan)


img1 = cv2.imread("C:/Users/Hayden/Desktop/Boundary.jpg")
#img1 = cv2.resize(img1, (0,0), fx=0.5, fy=0.5)
img2 = cv2.imread("C:/Users/Hayden/Desktop/Boundary2.jpg")
#img2 = cv2.resize(img2, (0,0), fx=0.5, fy=0.5)
img3 = cv2.imread("C:/Users/Hayden/Desktop/Boundary3.jpg")
#img3 = cv2.resize(img3, (0,0), fx=0.5, fy=0.5)
#cam0 = Camera(2, (1280,720), 6, (130,90), (0,45))
#cam0.create_angleAxis()

#dst = cv2.cornerHarris(gray,5,3,0.1)
def getCorners(img, slate):
	print img.shape
	print slate.shape, type(slate)
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	gray = np.uint8(gray)
		
	dst = cv2.cornerHarris(gray,2,3,0.04)

	dst = cv2.dilate(dst,None)

	#img[dst>0.01*dst.max()]=[0,0,255]
	#pic = img[dst>0.01*dst.max()]
	slate[dst>0.01*dst.max()]=[255,255,255]

	return slate

def eliminateFarCorners(xArray, yArray, maxDistance, cameraData):
	#Use camera data to convert maxdistance to y pixel
	print xArray.shape
	for i in range(len(xArray)):
		if yArray[i] >= maxDistance:
			np.delete(yArray[i],0)
			np.delete(xArray[i],0)





shape = img1.shape
ypix = shape[0]
xpix = shape[1]
tabulaRasa = np.zeros((ypix,xpix,3),np.uint8)


#pic2 = getCorners(img2)
#pic3 = getCorners(img3)

	#esc = Camera.showFrame(corner, 'corners')
	#cv2.imshow('im', im)cv2.imshow('img')
	#sc = cam0.showFrame(frame, 'poo')
	

#frame = getCorners(img)

cornerImg = getCorners(img1, tabulaRasa)
points = np.nonzero(cornerImg)
eliminate = eliminateFarCorners(points[0], points[1], 2, cornerImg)
cv2.imshow('pic', eliminate)
if cv2.waitKey(0) & 0xff== 27:
	cv2.destroyAllWindows()

print points
#cv2.imshow('dst2', pic2)
#cv2.imshow('dst3', pic3)
