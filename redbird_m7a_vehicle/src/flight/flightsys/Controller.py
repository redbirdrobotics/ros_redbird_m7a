#!/usr/bin/python

"""Controller.py: The controller serves as a state machine that handles communication with MAVROS setpoint types."""

import rospy
import threading
import math
import utils
import multiprocessing
from enum import Enum
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header

__author__ = "Alex Bennett"
__email__ = "alex.eugene.bennett@gmail.com"


class Control_Mode(Enum):
    INITIAL, TAKEOFF, HOLD, LAND, VELOCITY, POSITION = range(0, 6)


class Controller(object):
    def __init__(self, vehicle):
        # Define queue size
        self._queue_size = 10

        # Create publishers
        self._vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=self._queue_size)
        self._pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=self._queue_size)

        # Store vehicle reference
        self._vehicle = vehicle

        # Threading event
        self.event = threading.Event()

        # Setup variables
        self._log_tag = "[FC] "
        self.reset()

        # Set rospy rate
        self._rate = rospy.Rate(40)

        # Start thread
        try:
            # Create thread object
            self.thread = threading.Thread(target=self.loop, args=())

            # Start thread
            self.thread.start()

            # Allow thread to start
            self.event.wait(1)
        except:
            rospy.logerr("Unable to start flight control thread")

    def reset(self):
        # Reset variables
        self._takeoff_altitude = 2.0
        self._target_velocity = (0.0, 0.0, 0.0)
        self._target_position = (0.0, 0.0, 0.0)
        self._mode = Control_Mode.INITIAL

        # Reset thread event
        self.event.clear()

    def set_mode(self, mode):
        """Sets the vehicle mode to one of those available in the Control_Mode class.

        Args:
            mode (Control_Mode): The desired flight control mode.

        """
        # Raise exception if ROS is shutdown
        if rospy.is_shutdown():
            raise Exception("ROS shutdown")

        # Set mode
        self._mode = mode

        # Log info
        rospy.loginfo(self._log_tag + "Control mode changed to %s" % self._mode)

    def get_mode(self):
        """Returns the current Control_Mode of the controller."""
        return self._mode

    def set_velocity(self, vel, time=0.0):
        """Sets the desired velocity for the vehicle.

        The velocity is in the form of a tuple representing  the X, Y, and Z velocities (in m/s)
        and the time to remain at the given velocity (in sec).

        Args:
            vel (tuple): Velocity in the form (X, Y, Z).
            time (int): The time in seconds to maintain the velocity. 0 represents infinity. Default: 0

        """
        # Raise exception if ROS is shutdown
        if rospy.is_shutdown():
            raise Exception("ROS shutdown")

        # Verify parameter data types
        if type(vel) is not tuple or not all(isinstance(i, float) for i in vel) or type(time) is not float:
            raise TypeError("Invalid parameter data type(s)")

        # Set parameters
        self._target_velocity = vel
        self._travel_time = time

        # Log info
        rospy.loginfo(self._log_tag + "Target velocity set to %0.2f, %0.2f, %0.2f m/s" % self._target_velocity)
        if self._travel_time > 0: rospy.loginfo(self._log_tag + "Travel time set to %0.2f sec" % self._travel_time)

    def set_position(self, pos):
        """Sets the desired position for the vehicle.

        The position is in the form of a tuple representing the X, Y, and Z coordinates (in m).

        Args:
            pos (tuple): A tuple in for form (X, Y, Z).

        """
        # Raise exception if ROS is shutdown
        if rospy.is_shutdown():
            raise Exception("ROS shutdown")

        # Verify parameter data types
        if type(pos) is not tuple or not all(isinstance(i, float) for i in pos):
            raise TypeError("Invalid parameter data type(s)")

        # Set the position setpoint
        self._target_position = pos

        # Log info
        rospy.loginfo(self._log_tag + "Target position set to %0.2f, %0.2f, %0.2f m" % self._target_position)

    def set_takeoff_altitude(self, alt):
        """Sets the desired altitude to reach when taking off.s

        Args:
            alt (int): The altitude to takeoff to (in m).

        """
        # Raise exception if ROS is shutdown
        if rospy.is_shutdown():
            raise Exception("ROS shutdown")

        # Verify parameter data types
        if type(alt) is not float:
            raise TypeError("Invalid parameter data type(s)")

        # Set takeoff altitude
        self._takeoff_altitude = alt

        # Log info
        rospy.loginfo(self._log_tag + "Takeoff altitude set to %0.2f m" % self._takeoff_altitude)

    def get_takeoff_altitude(self):
        """Returns the set takeoff altitude."""
        return self._takeoff_altitude

    def loop(self):
        """This is the main loop that processes flight modes and enters the appropriate control loop."""
        try:
            # Enter loop indefinitely
            while not self.event.is_set() and not rospy.is_shutdown():
                if self._mode == Control_Mode.INITIAL:
                    # Wait for direction
                    pass
                elif self._mode == Control_Mode.TAKEOFF:
                    # Perform a takeoff and enter Control_Mode.HOLD
                    self.takeoff_loop()
                elif self._mode == Control_Mode.HOLD:
                    # Hold the current position
                    self.hold_loop()
                elif self._mode == Control_Mode.LAND:
                    # Perform a landing and return to Control_Mode.INITIAL
                    self.land_loop()
                elif self._mode == Control_Mode.VELOCITY:
                    # Enter the velocity control loop
                    self.velocity_loop()
                elif self._mode == Control_Mode.POSITION:
                    # Enter the position control loop
                    self.position_loop()
        except rospy.ROSException:
            pass

    def hold_loop(self):
        """Loops in Control_Mode.TAKEOFF until either the mode is switched or the target altitude has been reached."""

        # Grab current position
        current_position = self._vehicle.get_position()

        # Queue counter
        queue_count = 0

        # Loop while in Control_Mode.HOLD
        while not self.event.is_set() and not rospy.is_shutdown() and self._mode == Control_Mode.HOLD:
            # Build target position message
            msg = PoseStamped(header = Header(stamp=rospy.get_rostime()))
            msg.pose.position.x = current_position[0]
            msg.pose.position.y = current_position[1]
            msg.pose.position.z = current_position[2]

            # Publish message
            self._pos_pub.publish(msg)

            # Increment queue count
            queue_count += 1

            # Set to OFFBOARD mode after queue is filled with fresh data
            if queue_count >= self._queue_size:
                self._vehicle.set_mode('OFFBOARD')

            # Sleep to maintain appropriate update frequency
            self._rate.sleep()

    def land_loop(self):
        """Loops in Control_Mode.TAKEOFF until either the mode is switched or the target altitude has been reached."""

        # Count the number of iterations through the loop
        loops = 0

        # Queue counter
        queue_count = 0

        while not self.event.is_set() and not rospy.is_shutdown() and self._mode == Control_Mode.LAND:
            # Break if the number of loops has exceeded the required amount and the magnitude of movement is less than 0.1
            if loops >= 10 and \
                abs(math.sqrt(math.pow(self._vehicle.get_velocity_x(), 2) + \
                math.pow(self._vehicle.get_velocity_y(), 2) + \
                math.pow(self._vehicle.get_velocity_z(), 2))) < 0.1:
                break

            # Build and publish message
            msg = TwistStamped(header = Header(stamp=rospy.get_rostime()))
            msg.twist.linear.x = 0
            msg.twist.linear.y = 0
            msg.twist.linear.z = -0.25

            # Publish message
            self._vel_pub.publish(msg)

            # Increment queue count
            queue_count += 1

            # Set to OFFBOARD mode after queue is filled with fresh data
            if queue_count >= self._queue_size:
                self._vehicle.set_mode('OFFBOARD')

            # Sleep to maintain appropriate update frequency
            self._rate.sleep()

            # Increment number of loops
            loops += 1

        # Only perform if controller is still running
        if not self.event.is_set():
            # Set mode to INITIAL
            self._mode = Control_Mode.INITIAL

    def takeoff_loop(self):
        """Loops in Control_Mode.TAKEOFF until either the mode is switched or the target altitude has been reached."""

        # Queue counter
        queue_count = 0

        while not self.event.is_set() and not rospy.is_shutdown() and self._mode == Control_Mode.TAKEOFF:
            # Break if within allowed deviation of target altitude
            if utils.is_near((0, 0, self._vehicle.get_position_z()), (0, 0, self._takeoff_altitude), allowed_range=0.1):
                break

            # Build and publish message
            msg = TwistStamped(header = Header(stamp=rospy.get_rostime()))
            msg.twist.linear.x = 0
            msg.twist.linear.y = 0
            msg.twist.linear.z = 0.5

            # Publish message
            self._vel_pub.publish(msg)

            # Increment queue count
            queue_count += 1

            # Set to OFFBOARD mode after queue is filled with fresh data
            if queue_count >= self._queue_size:
                self._vehicle.set_mode('OFFBOARD')

            # Sleep to maintain appropriate update frequency
            self._rate.sleep()

        # Only perform if controller is still running
        if not self.event.is_set():
            # Set mode to HOLD
            self._mode = Control_Mode.HOLD

    def position_loop(self):
        """Loops in Control_Mode.POSITION until either the mode is switched or the target has been reached."""

        # Queue count
        queue_count = 0

        while not self.event.is_set() and not rospy.is_shutdown() and self._mode == Control_Mode.POSITION:
            # Break if within allowed deviation of target
            if utils.is_near(self._vehicle.get_position(), self._target_position):
                break

            # Build target position message
            msg = PoseStamped(header = Header(stamp=rospy.get_rostime()))
            msg.pose.position.x = self._target_position[0]
            msg.pose.position.y = self._target_position[1]
            msg.pose.position.z = self._target_position[2]

            # Publish message
            self._pos_pub.publish(msg)

            # Increment queue count
            queue_count += 1

            # Set to OFFBOARD mode after queue is filled with fresh data
            if queue_count >= self._queue_size:
                self._vehicle.set_mode('OFFBOARD')

            # Sleep to maintain appropriate update frequency
            self._rate.sleep()

        # Only perform if controller is still running
        if not self.event.is_set():
            # Set mode to HOLD
            self._mode = Control_Mode.HOLD

    def velocity_loop(self):
        """Loops in Control_Mode.VELOCITY until either the mode is switched or the set duration has been met."""

        # Queue counter
        queue_count = 0

        # Record start time for timeout tracking
        start_time = rospy.get_time()

        while not self.event.is_set() and not rospy.is_shutdown() and self._mode == Control_Mode.VELOCITY:
            # Break if timeout is enabled and has been surpassed
            if self._travel_time > 0 and (rospy.get_time() - start_time) > self._travel_time:
                break

            # Build and publish message
            msg = TwistStamped(header = Header(stamp=rospy.get_rostime()))
            msg.twist.linear.x = self._target_velocity[0]
            msg.twist.linear.y = self._target_velocity[1]
            msg.twist.linear.z = self._target_velocity[2]

            # Publish message
            self._vel_pub.publish(msg)

            # Increment queue count
            queue_count += 1

            # Set to OFFBOARD mode after queue is filled with fresh data
            if queue_count >= self._queue_size:
                self._vehicle.set_mode('OFFBOARD')

            # Sleep to maintain appropriate update frequency
            self._rate.sleep()

        # Only perform if controller is still running
        if not self.event.is_set():
            # Set mode to HOLD
            self._mode = Control_Mode.HOLD

            # Reset velocity target
            self.set_velocity((0.0, 0.0, 0.0))

    def wait_for_mode_change(self, mode):
        """Waits for the mode to change from what is passed in.

        Args:
            mode (Control_Mode): The mode to check for change from.
        """
        while self.get_mode() == mode:
            if rospy.is_shutdown():
                raise Exception("ROS shutdown")
