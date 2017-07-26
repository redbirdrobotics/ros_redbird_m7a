#!/usr/bin/python
import rospy
import cv2
import tf
import numpy as np
from redbird import *
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from cv_bridge import CvBridge, CvBridgeError
from redbird_m7a_msgs.msg import Goal, Goals


class Landmark_Localization(object):
    def __init__(self):
        # Create subscriber
        self.camera_sub = rospy.Subscriber('/redbird/localization/camera/image', Image, self.image_callback)
        self._position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)

        # Create publisher
        self._goal_pub = rospy.Publisher('/redbird/localization/goals', Goals, queue_size=10000)

        # Create running rate
        self._rate = rospy.Rate(10) # 20 Hz

        # Initialize quad information
        self.quadX = 0
        self.quadY = 0
        self.quadH = 0
        self.quadRoll = 0
        self.quadPitch = 0
        self.quadYaw = 0

        #Createblank image
        self._image = None

        # Create blank position state
        self._position_info = None

        # Create OpenCV bridge
        self._cv_bridge = CvBridge()

        # Camera Instances
        self.cam0 = Camera(0, (1280, 720),(130, 90), (0, 45))
        #self.cam1 = Camera(2, (1280, 720), 60, (130, 90), (180, 45))
        self.camList = [self.cam0]

        # Landmark Instances
        self.greengoal = Landmark(0, np.array([[79, 33, 66],[100, 111, 135]]))
        self.redgoal = Landmark(1, np.array([[161,145,64],[180,236,232]]))
        self.landmarkList = [self.greengoal, self.redgoal]

        # Lists
        self.redMaskList = []
        self.greenMaskList = []

    def image_callback(self, msg):
        try:
            self._image = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e

    def local_position_callback(self, msg):
        self.quadX = msg.pose.position.x
        self.quadY = msg.pose.position.y
        self.quadH = msg.pose.position.z

        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        self.quadRoll = euler[0]
        self.quadPitch = euler[1]
        self.quadYaw = euler[2]
        return

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self._image is None:
                    rospy.logdebug('[lm] no frame')
                    continue

                # Get Frame List
                frameList = [self._image]
                
                # Get Quad Data
                quadData = [self.quadX, self.quadY, self.quadH, self.quadYaw, self.quadPitch, self.quadRoll]

                # Get Mask Lists
                self.redgoal.getMaskList(frameList, self.redMaskList)
                self.greengoal.getMaskList(frameList, self.greenMaskList)

                # Search Masks
                self.redgoal.detectGoalLine(self.redMaskList, self.camList)
                self.greengoal.detectGoalLine(self.greenMaskList, self.camList)

                # Convert to meters
                self.redgoal.cvt2meters(quadData, self.camList)
                self.greengoal.cvt2meters(quadData, self.camList)

                
                # Create two goals (type Goal.msg)
                greengoal_msg = Goal(header=Header(stamp=rospy.get_rostime()))
                redgoal_msg = Goal(header=Header(stamp=rospy.get_rostime()))

                # Populate landmark information
                if self.greengoal.found == True:
                    greengoal_msg.color = self.greengoal.color
                    greengoal_msg.cam = self.greengoal.cam
                    greengoal_msg.x_m = [self.greengoal.mendPoints[0], self.greengoal.mendPoints[2]]
                    greengoal_msg.y_m = [self.greengoal.mendPoints[1], self.greengoal.mendPoints[3]]
                    greengoal_msg.x_px = [self.greengoal.endPoints[0], self.greengoal.endPoints[2]]
                    greengoal_msg.y_px = [self.greengoal.endPoints[1], self.greengoal.endPoints[3]]

                if self.redgoal.found == True:
                    redgoal_msg.color = self.redgoal.color
                    redgoal_msg.cam = self. redgoal.cam
                    redgoal_msg.x_m = [self.redgoal.mendPoints[0], self.redgoal.mendPoints[2]]
                    redgoal_msg.y_m = [self.redgoal.mendPoints[1], self.redgoal.mendPoints[3]]
                    redgoal_msg.x_px = [self.redgoal.endPoints[0], self.redgoal.endPoints[2]]
                    redgoal_msg.y_px = [self.redgoal.endPoints[1], self.redgoal.endPoints[3]]

                # Create goals message (type Goals.msg)
                goals_msg = Goals()
                goals_msg.goals = [greengoal_msg, redgoal_msg]

                # Publish to topic
                self._goal_pub.publish(goals_msg)

                #Edit & Show Frame ONLY FOR TESTING!!!!
                # self.greengoal.drawLine(self._image, 30)
                # self.redgoal.drawLine(self._image, 30)
                # Camera.showFrame(self._image, 'GoalFrame')

                # Match desired frequency
                self._rate.sleep()
            except Exception as e:
                 rospy.logwarn("[lm] Error: %s", str(e))
                 break


if __name__ == '__main__':
    try:
        # Setup ROS
        rospy.init_node('landmark_localization')

        # Create object
        localization = Landmark_Localization()

        # Run
        localization.run()
    except rospy.ROSException:
        pass

