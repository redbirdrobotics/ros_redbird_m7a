#!/usr/bin/python
import rospy
import cv2
import numpy as np
from redbird import *
#from GreenRobotLib import*
from sensor_msgs.msg import Image
#from redbird_m7a_msgs import FlightState.msg
from cv_bridge import CvBridge, CvBridgeError

class Green_Localization(object):
    def __init__(self):

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
            #Get Green Goal line endpoints

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self._image is None:
                    continue
                    print 'no frame'

                print 'working'

                # Function that would get current position of quad
                GreenRobot.listcvt2meters(0, 0, 1.524, self.foundList, self.camList)

                # Get image
                self.frameList = [self._image]

                # Replace following function with function that gets image list
                # Camera.getFrameList(camList, frameList)

                Utilities.getMaskList(self.frameList, self.GreenVals, self.maskList)

                # Get Landmark Node Data

                # Search ROI
                GreenRobot.ROIsearch(self.foundList, self.maskList, self.detector)
                GreenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                GreenRobot.listFound(self.robotList)

                # Search Whole Frame
                Utilities.blobSearch(self.maskList, self.detector, self.dataList, self.unfoundList)
                GreenRobot.listUpdate(self.foundList, self.unfoundList, self.dataList, self.camList)
                GreenRobot.sortFound(self.robotList, self.foundList, self.unfoundList)
                GreenRobot.listFound(self.robotList)

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
        localization = Green_Localization()

        # Run
        localization.run()
    except rospy.ROSException:
        pass










