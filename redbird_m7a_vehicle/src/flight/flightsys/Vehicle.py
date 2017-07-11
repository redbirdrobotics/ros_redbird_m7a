#!/usr/bin/python

# TODO: Add header docstring

import rospy
import mavros
import time
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped


class Vehicle(object):
    def __init__(self):
        # Set rate
        self._rate = rospy.Rate(10)

        # Initialize variables
        self._state_topic = State()
        self._local_position_topic = PoseStamped()
        self._local_velocity_topic = TwistStamped()

        # Wait for service startup
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')

        # Setup services
        self._set_mode_serv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self._arm_serv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        # Setup subscribers
        self._state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback)
        self._local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self._local_velocity_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.local_velocity_callback)

    ###############################
    ## Vehicle state information ##
    ###############################

    def arm(self):
        self._arm_serv(True)
        rospy.sleep(1)

    def disarm(self):
        self._arm_serv(False)

    def set_mode(self, mode):
        # If in AUTO.LAND, reject mode change
        if self.get_mode() != 'AUTO.LAND':
            # Set mode
            self._set_mode_serv(custom_mode=mode)

            # Allow a moment for change to propogate
            rospy.sleep(0.1)

            # Return true for success
            return True

        # Default false
        return False

    def is_connected(self):
        return self._state_topic.connected

    def is_armed(self):
        return self._state_topic.armed

    def get_mode(self):
        return self._state_topic.mode

    ################################
    ## Local position information ##
    ################################

    def get_position(self):
        return (self._local_position_topic.pose.position.x, self._local_position_topic.pose.position.y, self._local_position_topic.pose.position.z)

    def get_position_x(self):
        return self._local_position_topic.pose.position.x

    def get_position_y(self):
        return self._local_position_topic.pose.position.y

    def get_position_z(self):
        return self._local_position_topic.pose.position.z

    def get_orientation(self):
        return (self._local_position_topic.pose.orientation.x, self._local_position_topic.pose.orientation.y, self._local_position_topic.pose.orientation.z, self._local_position_topic.pose.orientation.w)

    def get_orientation_x(self):
        return self._local_position_topic.pose.orientation.x

    def get_orientation_y(self):
        return self._local_position_topic.pose.orientation.y

    def get_orientation_z(self):
        return self._local_position_topic.pose.orientation.z

    def get_orientation_w(self):
        return self._local_position_topic.pose.orientation.w

    ################################
    ## Local velocity information ##
    ################################

    def get_velocity(self):
        return (self._local_velocity_topic.twist.linear.x, self._local_velocity_topic.twist.linear.y, self._local_velocity_topic.twist.linear.z)

    def get_velocity_x(self):
        return self._local_velocity_topic.twist.linear.x

    def get_velocity_y(self):
        return self._local_velocity_topic.twist.linear.y

    def get_velocity_z(self):
        return self._local_velocity_topic.twist.linear.z

    ###############
    ## Callbacks ##
    ###############

    def state_callback(self, topic):
        self._state_topic = topic

    def local_position_callback(self, topic):
        self._local_position_topic = topic

    def local_velocity_callback(self, topic):
        self._local_velocity_topic = topic
