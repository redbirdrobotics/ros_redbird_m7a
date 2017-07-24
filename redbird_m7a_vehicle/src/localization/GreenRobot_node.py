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
        self._goals_sub = rospy.Subscriber('/redbird/localization/goals'), String, self.goals_callback

        # Create publisher
        self._greenrobot_pub = rospy.Publisher('/redbird/localization/greenrobots', greenrobotd, queue_size=10000)

        # Create running rate
        self._rate = rospy.Rate(10) # 20 Hz

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

        # Create blank landmark message
        self._goals_data = None

        # Create OpenCV bridge
        self._cv_bridge = CvBridge()

        # Camera Instances
        self.cam0 = Camera(0, (1280, 720), (130, 90), (0, 45.0))
        self.cam1 = Camera(1, (1280, 720), (130, 90), (180.0, 45.0))
        self.camList = [self.cam0]

        # GreenRobot Instances
        self.hulk = RedRobot(0)
        self.yoshi = RedRobot(1)
        self.yoda = RedRobot(2)
        self.arrow = RedRobot(3)
        self.beastboy = RedRobot(4)
        self.robotList = [self.hulk, self.yoshi, self.yoda, self.arrow, self.beastboy]

        # Landmark Instances
        self.greengoal = Landmark(0, np.array([[79, 33, 66],[100, 111, 135]]))

        # Threshold values
        self.greenVals = np.array([[165, 150, 150], [180, 240, 200]])

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

    def image_callback(self, msg):
        try:
            self._image = self._cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print e

   def flightdata_callback(self, msg):
        self.quadX = msg.pose.position.x
        self.quadY = msg.pose.position.y
        self.quadH = msg.pose.position.z

        quaternion = (self._local_position_topic.pose.orientation.x, self._local_position_topic.pose.orientation.y, self._local_position_topic.pose.orientation.z)
        euler = tf.transformation.euler_from_quaternion(quaternion)

        self.quadRoll = euler[0]
        self.quadPitch = euler[1]
        self.quadYaw = euler[2]
        return

    def goals_callback(self, msg):
        self.greengoal.endPoints = (msg.x_px[0], msg.y_px[0], msg.x_px[1], msg.y_px[1])
        self.greengoal.cam = msg.cam
                greenrobot_msg.number = robot.num
                greenrobot_msg.color = robot.color
                greenrobot_msg.coords_m = robot.mcoords
                greenrobot_msg.vector = robot.vector
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

                # Function to remove landmarks from node data
                self.greengoal.remove(self.maskList, 30)

                # Search ROI
                greenRobot.ROIsearch(self.foundList, self.maskList, self.detector)
                greenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                greenRobot.listFound(self.robotList)

                # Search Whole Frame
                Utilities.blobSearch(self.maskList, self.detector, self.dataList, self.unfoundList)
                greenRobot.listUpdate(self.foundList, self.unfoundList, self.dataList, self.camList)
                greenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                greenRobot.listFound(self.robotList)

                #Create greenrobot Message
                greenrobot_msg = GroundRobotPosition(header=Header(stamp=rospy.get_rostime())

                # Populate Robot information
                for i in range(len(self.robotList)):
                    greenrobot_msg.GreenRobotPosition[i].id = self.robotList[i].num
                    greenrobot_msg.GreenRobotPosition[i].x = self.robotList[i].coords[0]
                    greenrobot_msg.GreenRobotPosition[i].y = self.robotList[i].coords[1]
                    greenrobot_msg.GreenRobotPosition[i].vec_x = self.robotList[i].vector[0]
                    greenrobot_msg.GreenRobotPosition[i].vec_y = self.robotList[i].vector[1]
                    greenrobot_msg.GreenRobotPosition[i].color = 0

                # Publish to topic
                self._greenrobot_pub.publish(greenrobot_msg)

                # Match desired frequency
                self._rate.sleep()

                # Testing
                frame = Utilities.circleFound(self.frameList[0], self.foundList)

                #Show Frame
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
