#!/usr/bin/python
import rospy
import cv2
import numpy as np
from redbird import*
from sensor_msgs.msg import Image
#from redbird_m7a_msgs import FlightState.msg
from cv_bridge import CvBridge, CvbridgeError

class Landmark_Localization(object):
	def__init__(self):
	# Create subscriber
	self.camera_sub = rospy.Subscriber('/redbird/localization/camera/image', Image, self.image_callback)
	self._position_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.local_position_callback)

	#Createblank image
	self._image = None

	# Create blank position state
	self._position_info = None

	# Create OpenCV bridge
	self._cv_bridge = CvBridge()

	# Camera Instances
	self.cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 45))
	#self.cam1 = Camera(2, (1280, 720), 60, (130, 90), (180, 45))
	self.camList = [self.cam0]

	# Landmark Instances
	self.redgoal = Landmark(np.array([[161,145,64],[180,236,232]]))
	self.greengoal = Landmark(np.array([[79, 33, 66],[100, 111, 135]]))
	self.landmarkList[self.redgoal, self.greengoal]

	# Lists
	self.redMaskList = []
	self.greenMaskList = []

def image_callback(self, msg):
	try:
		self.image = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvbridgeError as e:
		print e

def flightdata_callback(self, msg):
	self.quadX = self._local_position_topic.pose.position.x
	self.quadY = self._local_position_topic.pose.position.y
	self.quadH = self._local_position_topic.pose.position.z

	quaternion = (self._local_position_topic.pose.orientation.x, self._local_position_topic.pose.orientation.y, self._local_position_topic.pose.orientation.z)
	euler = tf.transformation.euler_from_quaternion(quaternion)

	self.quadRoll = euler[0]
	self.quadPitch = euler[1]
	self.quadYaw = euler[2]
	return

def run(self):
	while not rospy.is_shutdown():
		try:
			if self._image is None:
				continue
				print 'no frame'

			print 'working'

			# Get Frame List
			self.frameList = [self.image]

			# Get Quad Data
			self.quadData[self.quadX, self,quadY, self.quadH, self.quadYaw, self.quadPitch, self.quadRoll]

			#Get Mask Lists
			self.redgoal.getMaskList(self.frameList, self.redMaskList)
			self.greengoal.getMaskList(self.frameList, self.greenMaskList)

			#Search Masks
			self.redgoal.detectGoalLine(self.redMaskList)
			self.greengoal.detectGoalLine(self.greenList)

			# Convert to meters
			self.redgoal.cv2meters(self.quadData, self.camList)
			self.greengoal.cv2meters(self.quadData, self.camList)



    #Publish to Node

    #Show Frame
    

