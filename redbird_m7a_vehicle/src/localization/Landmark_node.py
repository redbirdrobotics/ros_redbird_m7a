#!/usr/bin/python
import rospy
import cv2
import numpy as np
from redbird import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from redbird_m7a_msgs.msg import Goal, Goals


class Landmark_Localization(object):
    def __init__(self):
        # Create subscriber
        self.camera_sub = rospy.Subscriber('/redbird/localization/camera/image', Image, self.image_callback)
        self._position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)

        # Create publisher
        self._goal_pub = rospy.Publisher('/redbird/localization/goals', Goals, queue_size=1)

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
        self.landmarkList[self.redgoal, self.greengoal]

        # Lists
        self.redMaskList = []
        self.greenMaskList = []

    def image_callback(self, msg):
        try:
            self.image = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvbridgeError as e:
            print e

    def local_position_callback(self, msg):
        self.quadX = msg.pose.position.x
        self.quadY = msg.pose.position.y
        self.quadH = msg.pose.position.z

        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
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

                # Get Mask Lists
                self.redgoal.getMaskList(self.frameList, self.redMaskList)
                self.greengoal.getMaskList(self.frameList, self.greenMaskList)

                # Search Masks
                self.redgoal.detectGoalLine(self.redMaskList)
                self.greengoal.detectGoalLine(self.greenList)

                # Convert to meters
                self.redgoal.cv2meters(self.quadData, self.camList)
                self.greengoal.cv2meters(self.quadData, self.camList)

                # Create two goals (type Goal.msg)
                goal_msgs = [Goal(), Goal()]

                # Populate landmark information
                goal_msgs[0].color = self.greengoal.color
                goal_msgs[0].x_m = [self.greengoal.endPoints[0], self.greengoal.endPoints[2]]
                goal_msgs[0].y_m = [self.greengoal.endPoints[1], self.greengoal.endPoints[3]]
                goal_msgs[0].x_px = [self.greengoal.mendPoints[0], self.greengoal.mendPoints[2]]
                goal_msgs[0].y_px = [self.greengoal.mendPoints[1], self.greengoal.mendPoints[3]]

                goal_msgs[1].color = self.redgoal.color
                goal_msgs[1].x_m = [self.redgoal.endPoints[0], self.redgoal.endPoints[2]]
                goal_msgs[1].y_m = [self.redgoal.endPoints[1], self.redgoal.endPoints[3]]
                goal_msgs[1].x_px = [self.redgoal.mendPoints[0], self.redgoal.mendPoints[2]]
                goal_msgs[1].y_px = [self.redgoal.mendPoints[1], self.redgoal.mendPoints[3]]

                # Create goals message (type Goals.msg)
                goals_msg = Goals()
                goals_msg.goals = goal_msgs

                # Publish to topic
                self._goal_pub.publish(goals_msg)

                #Show Frame, Only for Testing
                Camera.showFrame(self.image, 'GoalFrame')

            except Exception as e:
                rospy.logwarn("Error: %s", str(e))
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

