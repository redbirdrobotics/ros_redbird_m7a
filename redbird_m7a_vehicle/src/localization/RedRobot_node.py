#!/usr/bin/python
import rospy
import cv2
import numpy as np
from redbird import *
#from RedRobotLib import*
from sensor_msgs.msg import Image
#from redbird_m7a_msgs import FlightState.msg
from cv_bridge import CvBridge, CvBridgeError

class Red_Localization(object):
    def __init__(self):
        # Create subscriber
        self._sub = rospy.Subscriber('/redbird/localization/camera/image', Image, self.image_callback)
        #self._sub = rospy.Subscriber('/redbird/mavros', String, self.flightdata_callback)
        #self._sub = rospy.Subscriber('/redbird/localization/landmark'), String, self.landmark_callback)

        # Create blank image
        self._image = None

        # Create OpenCV bridge
        self._cv_bridge = CvBridge()

        # CAMERAS TO BE REPLACES BY GSTREAMER FUNCTION
        self.cam0 = Camera(1, (1280, 720), 60, (130, 90), (0, 53.7))
        self.camList = [self.cam0]

        # RED ROBOTS
        self.daredevil = RedRobot(0)
        self.deadpool = RedRobot(1)
        self.elmo = RedRobot(2)
        self.hellboy = RedRobot(3)
        self.flash = RedRobot(4)
        self.robotList = [self.daredevil, self.deadpool, self.elmo, self.hellboy, self.flash]

        # THRESHOLD
        self.redVals = np.array([[165, 150, 150], [180, 240, 200]])

        # BLOB DETECTOR
        self.RedRobotParams = cv2.SimpleBlobDetector_Params()
        Utilities.getParams(self.RedRobotParams, 0)
        self.detector = cv2.SimpleBlobDetector_create(self.RedRobotParams)

        # LISTS
        self.foundList = []
        self.unfoundList = []
        self.frameList = []
        self.maskList = []
        self.dataList = []

    def image_callback(self, msg):
        try:
            self._image = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e

    #def flightdata_callback(self, msg):
        #try:
            #Get Yaw, Pitch Roll, altitude.

    #def landmark_callback(self, msg):
        #try:
            #Get Red Goal line endpoints

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self._image is None:
                    continue
                    print 'no frame'

                print 'working'

                ###### Function that would get current position of quad
                RedRobot.listcvt2meters(0, 0, 1.524, self.foundList, self.camList)

                # Get image
                self.frameList = [self._image]

                # Replace following function with function that gets image list
                # Camera.getFrameList(camList, frameList)

                Utilities.getMaskList(self.frameList, self.redVals, self.maskList)

                # Get Landmark Node Data

                # Search ROI
                RedRobot.ROIsearch(self.foundList, self.maskList, self.detector)
                RedRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                RedRobot.listFound(self.robotList)

                # Search Whole Frame
                Utilities.blobSearch(self.maskList, self.detector, self.dataList, self.unfoundList)
                RedRobot.listUpdate(self.foundList, self.unfoundList, self.dataList, self.camList)
                RedRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                RedRobot.listFound(self.robotList)

                # Testing
                frame = Utilities.circleFound(self.frameList[0], self.foundList)

                # Function to remove landmarks from node data
                esc = Camera.showFrame(frame, 'frame')
                if esc == True:
                    break
                    
            except Exception as e:
                rospy.logwarn("Error: %s", str(e))
                break

if __name__ == '__main__':
    try:
        # Setup ROS
        rospy.init_node('red_localization')

        # Create object
        localization = Red_Localization()

        # Run
        localization.run()
    except rospy.ROSException:
        pass










