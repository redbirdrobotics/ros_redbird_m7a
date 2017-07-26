#!/usr/bin/python
import rospy
import cv2
import tf
import numpy as np
from redbird import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from redbird_m7a_msgs.msg import GreenRobotMap, GroundRobotPosition, Goals
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from std_msgs.msg import Header


class Green_Localization(object):
    def __init__(self):
        # Create subscriber
        self._camera_sub = rospy.Subscriber('/redbird/localization/camera/image', Image, self.image_callback)
        self._position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self._goals_sub = rospy.Subscriber('/redbird/localization/goals', Goals, self.goals_callback)

        # Create publisher
        self._greenrobot_pub = rospy.Publisher('/redbird/localization/robots/green', GreenRobotMap, queue_size=10000)

        # Create running rate
        self._rate = rospy.Rate(10) # 10 Hz

        # Initialize quad information
        self.quadX = 0
        self.quadY = 0
        self.quadH = 0
        self.quadRoll = 0
        self.quadPitch = 0
        self.quadYaw = 0

        # Create blank image
        self._image = None

        # Create blank position state
        self._position_info = None

        # Create OpenCV bridge
        self._cv_bridge = CvBridge()

        # Camera Instances
        self.cam0 = Camera(0, (1280, 720), (130, 90), (0, 45.0))
        self.cam1 = Camera(1, (1280, 720), (130, 90), (180.0, 45.0))
        self.camList = [self.cam0, self.cam1]

        # GreenRobot Instances
        self.hulk = GreenRobot(0)
        self.yoshi = GreenRobot(1)
        self.yoda = GreenRobot(2)
        self.arrow = GreenRobot(3)
        self.beastboy = GreenRobot(4)
        self.robotList = [self.hulk, self.yoshi, self.yoda, self.arrow, self.beastboy]

        # Landmark Instances
        self.greengoal = Landmark(0, np.array([[79, 33, 66],[100, 111, 135]]))

        # Threshold values
        self.greenVals = np.array([[70, 81, 78], [94, 241, 154]])

        # Blob Detector
        self.GreenRobotParams = cv2.SimpleBlobDetector_Params()
        Utilities.getParams(self.GreenRobotParams, 0)
        self.detector = cv2.SimpleBlobDetector_create(self.GreenRobotParams)

        # Empty Lists
        self.foundList = []
        self.unfoundList = []
        self.frameList = []
        self.maskList = []
        self.dataList = []

        rospy.sleep(2)

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

    def goals_callback(self, msg):
        for goal in msg.goals:
            if goal.color == 0:
                self.greengoal.endPoints = (goal.x_px[0], goal.y_px[0], goal.x_px[1], goal.y_px[1])
                self.greengoal.cam = goal.cam

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self._image is None:
                    rospy.logdebug('[gr] no frame')
                    continue

                # Get Quad Data
                self.quadData = [self.quadX, self.quadY, self.quadH, self.quadYaw, self.quadPitch, self.quadRoll]

                GreenRobot.listcvt2meters(self.quadData, self.foundList, self.camList)

                # Get image
                self.frameList = [self._image]

                Utilities.getMaskList(self.frameList, self.greenVals, self.maskList)

                # Function to remove landmarks from node data
                if self.greengoal.cam is not None:
                    self.greengoal.remove(self.maskList, 30)

                # Search ROI
                GreenRobot.ROIsearch(self.foundList, self.maskList, self.detector)
                GreenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                GreenRobot.listFound(self.robotList)

                # Search Whole Frame
                Utilities.blobSearch(self.maskList, self.detector, self.dataList, self.unfoundList)
                GreenRobot.listUpdate(self.foundList, self.unfoundList, self.dataList, self.camList)
                GreenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                GreenRobot.listFound(self.robotList)

                #Create greenrobot Message
                green_robot_msgs = []

                # Populate Robot information
                for i in range(len(self.robotList)):
                    green_robot_msgs.append(GroundRobotPosition(header=Header(stamp=rospy.get_rostime())))
                    green_robot_msgs[i].id = self.robotList[i].ident
                    green_robot_msgs[i].x = self.robotList[i].mcoords[0]
                    green_robot_msgs[i].y = self.robotList[i].mcoords[1]
                    green_robot_msgs[i].vec_x = self.robotList[i].vector[0]
                    green_robot_msgs[i].vec_y = self.robotList[i].vector[1]
                    green_robot_msgs[i].color = 0
                    green_robot_msgs[i].confidence = 1.0
                    green_robot_msgs[i].out_of_bounds = False

                # Create green robot map
                green_robot_map_msg = GreenRobotMap(header=Header(stamp=rospy.get_rostime()))   
                green_robot_map_msg.robots = green_robot_msgs

                # Publish to topic
                self._greenrobot_pub.publish(green_robot_map_msg)

                # Match desired frequency
                self._rate.sleep()

                # Testing
                frame = Utilities.circleFound(self.frameList[0], self.foundList)
                self.greengoal.drawLine(frame, 30)

                #Show Frame
                # esc = Camera.showFrame(frame, 'frame')
                # if esc == True:
                #     break

            except Exception as e:
                rospy.logwarn("[gr] Error: %s", str(e))
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
