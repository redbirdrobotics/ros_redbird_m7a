#!/usr/bin/python

# TODO: Add header docstring

import rospy
import threading
import math
import utils
from enum import Enum
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header


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

        # Setup variables
        self._log_tag = "[FC] "
        self._takeoff_altitude = 2.0
        self._target_velocity = (0.0, 0.0, 0.0)
        self._target_position = (0.0, 0.0, 0.0)

        # Set rospy rate
        self._rate = rospy.Rate(20)

        # Threading
        self._thread_done = threading.Event()

        # State machine
        self._mode = Control_Mode.INITIAL

        # Handle threading
        try:
            # Create thread object
            self._thread = threading.Thread(target=self.loop, args=())

            # Start thread
            self._thread.start()

            # Allow thread to start
            rospy.sleep(1)
        except:
            rospy.logerror("Unable to start flight control thread")

    def set_mode(self, mode):
        """Sets the vehicle mode to one of those available in the Control_Mode class.

        Args:
            mode (Control_Mode): The desired flight control mode.

        """
        # Set mode
        self._mode = mode

        # Log info
        rospy.loginfo(self._log_tag + "Mode changed to %s" % self._mode)

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
        # Verify parameter data types
        if type(vel) is not tuple or not all(isinstance(i, float) for i in vel) or type(time) is not float:
            raise TypeError("Invalid parameter data type(s)")

        # Set parameters
        self._target_velocity = vel
        self._travel_time = time

        # Log info
        rospy.loginfo(self._log_tag + "Target velocity set to %s m/s" % (self._target_velocity,))
        if self._travel_time > 0: rospy.loginfo(self._log_tag + "Travel time set to %0.2f sec" % self._travel_time)

    def set_position(self, pos):
        """Sets the desired position for the vehicle.

        The position is in the form of a tuple representing the X, Y, and Z coordinates (in m).

        Args:
            pos (tuple): A tuple in for form (X, Y, Z).

        """
        # Verify parameter data types
        if type(pos) is not tuple or not all(isinstance(i, float) for i in pos):
            raise TypeError("Invalid parameter data type(s)")

        # Set the position setpoint
        self._target_position = pos

        # Log info
        rospy.loginfo(self._log_tag + "Target position set to %s m" % (self._target_position,))

    def set_takeoff_altitude(self, alt):
        """Sets the desired altitude to reach when taking off.

        Args:
            alt (int): The altitude to takeoff to (in m).

        """
        # Verify parameter data types
        if type(alt) is not float:
            raise TypeError("Invalid parameter data type(s)")

        # Set takeoff altitude
        self._takeoff_altitude = alt

        # Log info
        rospy.loginfo(self._log_tag + "Takeoff altitude set to %0.2f m" % self._takeoff_altitude)

    def loop(self):
        """This is the main loop that processes flight modes and enters the appropriate control loop."""

        # Enter loop indefinitely
        while not rospy.is_shutdown():
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

    def hold_loop(self):
        """Loops in Control_Mode.TAKEOFF until either the mode is switched or the target altitude has been reached."""

        # Grab current position
        current_position = self._vehicle.get_position()

        # Queue counter
        queue_count = 0

        # Loop while in Control_Mode.HOLD
        while not rospy.is_shutdown() and self._mode == Control_Mode.HOLD:
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

        while not rospy.is_shutdown() and self._mode == Control_Mode.LAND:
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

        # Set mode to HOLD
        self.set_mode(Control_Mode.INITIAL)

    def takeoff_loop(self):
        """Loops in Control_Mode.TAKEOFF until either the mode is switched or the target altitude has been reached."""

        # Queue counter
        queue_count = 0

        while not rospy.is_shutdown() and self._mode == Control_Mode.TAKEOFF:
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

        # Set mode to HOLD
        self.set_mode(Control_Mode.HOLD)

    def position_loop(self):
        """Loops in Control_Mode.POSITION until either the mode is switched or the target has been reached."""

        # Queue count
        queue_count = 0

        while not rospy.is_shutdown() and self._mode == Control_Mode.POSITION:
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

        # Set mode to HOLD
        self.set_mode(Control_Mode.HOLD)

    def velocity_loop(self):
        """Loops in Control_Mode.VELOCITY until either the mode is switched or the set duration has been met."""

        # Queue counter
        queue_count = 0

        # Record start time for timeout tracking
        start_time = rospy.get_time()

        while not rospy.is_shutdown() and self._mode == Control_Mode.VELOCITY:
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

        # Set mode to HOLD
        self.set_mode(Control_Mode.HOLD)

        # Reset velocity target
        self.set_velocity((0.0, 0.0, 0.0))
