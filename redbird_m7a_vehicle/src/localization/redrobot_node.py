#!/usr/bin/python
import rospy
import cv2
import tf
import numpy as np
from redbird import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from redbird_m7a_msgs.msg import RedRobotMap, GroundRobotPosition, Goals
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from std_msgs.msg import Header


class Red_Localization(object):
    def __init__(self):
        # Create subscriber
        self._camera_sub = rospy.Subscriber('/redbird/localization/camera/image', Image, self.image_callback)
        self._position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self._sub = rospy.Subscriber('/redbird/localization/goals', Goals, self.goals_callback)

        # Create publisher
        self._redrobot_pub = rospy.Publisher('/redbird/localization/robots/red', RedRobotMap, queue_size=10000)

        # Create running rate
        self._rate = rospy.Rate(10) # 10 Hz

        # Initialize Quad Information
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
        self.cam0 = Camera(1, (1280, 720), (130, 90), (0, 45.0))
        self.cam1 = Camera(1, (1280, 720), (130, 90), (180, 45.0))
        self.camList = [self.cam0, self.cam1]

        # RedRobot Instances
        self.daredevil = RedRobot(0)
        self.deadpool = RedRobot(1)
        self.elmo = RedRobot(2)
        self.hellboy = RedRobot(3)
        self.flash = RedRobot(4)
        self.robotList = [self.daredevil, self.deadpool, self.elmo, self.hellboy, self.flash]

        # Landmark Instances
        self.redgoal = Landmark(1, np.array([[161, 145, 64], [180, 236, 232]]))

        # Threshold Values
        self.redVals = np.array([[165, 150, 150], [180, 240, 200]])

        # Blob Detector
        self.RedRobotParams = cv2.SimpleBlobDetector_Params()
        Utilities.getParams(self.RedRobotParams, 0)
        self.detector = cv2.SimpleBlobDetector_create(self.RedRobotParams)

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
        return

    def goals_callback(self, msg):
        for goal in msg.goals:
            if goal.color == 0:
                self.redgoal.endPoints = (goal.x_px[0], goal.y_px[0], goal.x_px[1], goal.y_px[1])
                self.redgoal.cam = goal.cam

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self._image is None:
                    rospy.logdebug('[rr] no frame')
                    continue

                # Get Quad Data
                self.quadData= [self.quadX, self.quadY, self.quadH, self.quadYaw, self.quadPitch, self.quadRoll]

                RedRobot.listcvt2meters(self.quadData, self.foundList, self.camList)

                # Get image
                self.frameList = [self._image]

                # Create Mask
                Utilities.getMaskList(self.frameList, self.redVals, self.maskList)

                # Remove Landmarks From Mask
                if self.redgoal.cam is not None:
                    self.redgoal.remove(self.maskList, 30)

                # Search ROI
                RedRobot.ROIsearch(self.foundList, self.maskList, self.detector)
                RedRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                RedRobot.listFound(self.robotList)

                # Search Whole Frame
                Utilities.blobSearch(self.maskList, self.detector, self.dataList, self.unfoundList)
                RedRobot.listUpdate(self.foundList, self.unfoundList, self.dataList, self.camList)
                RedRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                RedRobot.listFound(self.robotList)

                # Create Redrobot Message
                red_robot_msgs = []

                # Populate Robot Information
                for i in range(len(self.robotList)):
                    red_robot_msgs.append(GroundRobotPosition(header=Header(stamp=rospy.get_rostime())))
                    red_robot_msgs[i].id = self.robotList[i].ident
                    red_robot_msgs[i].x = self.robotList[i].mcoords[0]
                    red_robot_msgs[i].y = self.robotList[i].mcoords[1]
                    red_robot_msgs[i].vec_x = self.robotList[i].vector[0]
                    red_robot_msgs[i].vec_y = self.robotList[i].vector[1]
                    red_robot_msgs[i].color = 1
                    red_robot_msgs[i].confidence = 1.0
                    red_robot_msgs[i].out_of_bounds = False

                # Create Red Robot Map
                red_robot_map_msg = RedRobotMap(header=Header(stamp=rospy.get_rostime()))
                red_robot_map_msg.robots = red_robot_msgs

                # Publish to Topic
                self._redrobot_pub.publish(red_robot_map_msg)

                # Match Desired frequency
                self._rate.sleep()

                # Testing
                frame = Utilities.circleFound(self.frameList[0], self.foundList)
                self.redgoal.drawLine(frame, 30)

                # esc = Camera.showFrame(frame, 'frame')
                # if esc == True:
                #     break
                    
            except Exception as e:
                rospy.logwarn("[rr] Error: %s", str(e))
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










