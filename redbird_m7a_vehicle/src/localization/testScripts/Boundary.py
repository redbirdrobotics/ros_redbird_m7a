import cv2
import time
import numpy as np
from redbird import Camera, Utilities
import math
import os
#1280x720

#Camera
cam0 = Camera(2, (1280,720), 60, (130,90), (0,45) )
cam0.create_angleAxis()

def get_pixels(frame, height):
	
	isBoundary = False
	xmin = 1280
	xmax = 0
	ymin = 720
	ymax = 0
	x = np.array([0])

	gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
	edges = cv2.Canny(gray,50,150,apertureSize = 3)

	lines = cv2.HoughLinesP(edges, 1,np.pi/180,150)

	if np.all(lines == None):
		return False

	for line in lines:
		
		for x1, y1, x2, y2 in line:
			
			if (x1 < xmin):
				xmin = x1
			if (x2 < xmin):
				xmin = x2
			if (x1 > xmax):
				xmax = x1
			if (x2 > xmax):
				xmax = x2


			if (y1 < ymin):
				ymin = y1
			if (y2 < ymin):
				ymin = y2
			if (y1 > ymax):
				ymax = y1
			if (y2 > ymax):
				ymax = y2
	

	#x = cam0.xAxis
	#y = cam0.yAxiz

	(dx1,dy1) = Utilities.get_dis(cam0.xAxis,cam0.yAxis,(xmin,ymin),height)
	(dx2,dy2) = Utilities.get_dis(cam0.xAxis,cam0.yAxis,(xmin,ymax),height)
	(dx3,dy3) = Utilities.get_dis(cam0.xAxis,cam0.yAxis,(xmax,ymin),height)
	(dx4,dy4) = Utilities.get_dis(cam0.xAxis,cam0.yAxis,(xmax,ymax),height)

	d1 = math.sqrt(dx1*dx1+dy1*dy1)
	d2 = math.sqrt(dx2*dx2+dy2*dy2)
	d3 = math.sqrt(dx3*dx3+dy3*dy3)
	d4 = math.sqrt(dx4*dx4+dy4*dy4)

	if((d1 <= 2) or (d2<= 2) or (d3<=2) or (d4 <= 2)):
		isBoundary = True
	else:
		isBoundary = False

	return isBoundary

while True:
	img = cam0.getFrame() 
	print get_pixels(img, 1)
	esc = cam0.showFrame(img, 'poo')
	if esc == True:
		break

cam0.detach()