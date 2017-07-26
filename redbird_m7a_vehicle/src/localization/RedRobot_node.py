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
        self.cam0 = Camera(1, (1280, 720), (130, 90), (0, 53.7))
        self.camList = [self.cam0]

        # RedRobot Instances
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
            #Get Red Goal line endpoints

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self._image is None:
                    continue
                    print 'no frame'

                #print 'working'

                # Get Quad Data
                self.quadData= [self.quadX, self,quadY, self.quadH, self.quadYaw, self.quadPitch, self.quadRoll]

                RedRobot.listcvt2meters(0, 0, 1.524, self.foundList, self.camList)

                # Get image
                self.frameList = [self._image]

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










