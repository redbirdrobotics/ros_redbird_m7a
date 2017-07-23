#!/usr/bin/python
import rospy
import cv2
import numpy as np
from redbird import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Red_Localization(object):


    def __init__(self):

        # Create subscriber
        self._camera_sub = rospy.Subscriber('/redbird/localization/camera/image', Image, self.image_callback)
        self._position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        #self._sub = rospy.Subscriber('/redbird/localization/landmark'), String, self.landmark_callback)

        # Create blank image
        self._image = None

        # Create blank position state
        self._position_info = None

        # Create OpenCV bridge
        self._cv_bridge = CvBridge()

        # Camera Instances
        self.cam0 = Camera(0, (1280, 720), (130, 90), (0, 45.0))
        self.cam1 = Camera(1, (1280, 720), (130, 90), (180.0, 45.0))
        self.camList = [self.cam0]

        # RedRobot Instances
        self.hulk = RedRobot(0)
        self.yoshi = RedRobot(1)
        self.yoda = RedRobot(2)
        self.arrow = RedRobot(3)
        self.beastboy = RedRobot(4)
        self.robotList = [self.hulk, self.yoshi, self.yoda, self.arrow, self.beastboy]

        # THRESHOLD
        self.greenVals = np.array([[165, 150, 150], [180, 240, 200]])

        # BLOB DETECTOR
        self.GreenRobotParams = cv2.SimpleBlobDetector_Params()
        Utilities.getParams(self.GreenRobotParams, 0)
        self.detector = cv2.SimpleBlobDetector_create(self.GreenRobotParams)

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

    #def landmark_callback(self, msg):
        #try:
            #Get Green Goal line endpoints

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self._image is None:
                    continue
                    print 'no frame'

                print 'working'

                # Get Quad Data
                self.quadData= [self.quadX, self,quadY, self.quadH, self.quadYaw, self.quadPitch, self.quadRoll]

                GreenRobot.listcvt2meters(self.quadData, self.foundList, self.camList)

                # Get image
                self.frameList = [self._image]

                Utilities.getMaskList(self.frameList, self.greenVals, self.maskList)

                # Get Landmark Node Data

                # Search ROI
                greenRobot.ROIsearch(self.foundList, self.maskList, self.detector)
                greenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                greenRobot.listFound(self.robotList)

                # Search Whole Frame
                Utilities.blobSearch(self.maskList, self.detector, self.dataList, self.unfoundList)
                greenRobot.listUpdate(self.foundList, self.unfoundList, self.dataList, self.camList)
                greenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                greenRobot.listFound(self.robotList)

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
        rospy.init_node('green_localization')

        # Create object
        localization = Green_Localization()

        # Run
        localization.run()
    except rospy.ROSException:
        pass
