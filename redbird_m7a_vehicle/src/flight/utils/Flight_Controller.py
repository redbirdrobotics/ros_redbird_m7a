import rospy
import threading
import time
from geometry_msgs.msg import TwistStamped, PoseStamped

class Flight_Mode:
    INITIAL, TAKEOFF, HOLD, LAND, VELOCITY, POSITION = range(0, 5)

class Flight_Controller:
    def __init__(self, vehicle):
        # Create publishers
        self._vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self._pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Store vehicle reference
        self._vehicle = vehicle

        # Setup variables
        self._takeoff_altitude = 2
        self._target_velocity = (0, 0, 0)
        self._target_position = (0, 0, 0)
        self._target_position_reached = False

        # Set rospy rate
        self._rate = rospy.Rate(20)

        # Threading
        self._thread_done = threading.Event()

        # State machine
        self._mode = Flight_Mode.INITIAL

        # Handle threading
        try:
            # Create thread object
            self._thread = threading.Thread(target=self.loop, args=())

            # Start thread
            self._thread.start()

            # Allow thread to start
            time.sleep(1)
        except:
            rospy.logerror("Unable to start flight control thread")

    def set_mode(self, mode):
        """Sets the vehicle mode to one of those available in the Flight_Mode class.

        Args:
            mode (Flight_Mode): The desired flight control mode.

        """
        self._mode = mode

    def set_velocity(self, vel, time):
        """Sets the desired velocity for the vehicle.

        The velocity is in the form of a tuple representing  the X, Y, and Z velocities (in m/s)
        and the time to remain at the given velocity (in sec).

        Args:
            vel (tuple): Velocity in the form (X, Y, Z).
            time (int): The time in seconds to maintain the velocity.

        """
        self._target_velocity = vel

    def set_position(self, pos):
        """Sets the desired position for the vehicle.

        The position is in the form of a tuple representing the X, Y, and Z coordinates (in m).

        Args:
            pos (tuple): A tuple in for form (X, Y, Z).

        """
        # Reset the reached position flag
        self._target_position_reached = False

        # Set the position setpoint
        self._target_position = pos

    def set_takeoff_altitude(self, alt):
        """Sets the desired altitude to reach when taking off.

        Args:
            alt (int): The altitude to takeoff to (in m).

        """
        self._takeoff_altitude = alt

    def loop(self):
        """This is the main loop that processes flight modes and enters the appropriate control loop."""

        # Enter loop indefinitely
        while not rospy.is_shutdown():
            if self._mode == Flight_Mode.INITIAL:
                # Setup flight and wait for direction
                pass
            elif self._mode == Flight_Mode.TAKEOFF:
                # Perform a takeoff and enter Flight_Mode.HOLD
                pass
            elif self._mode == Flight_Mode.HOLD:
                # Hold the current position
                pass
            elif self._mode == Flight_Mode.LAND:
                # Perform a landing and return to Flight_Mode.INITIAL
                pass
            elif self._mode == Flight_Mode.VELOCITY:
                # Enter the velocity control loop
                pass
            elif self._mode == Flight_Mode.POSITION:
                # Enter the position control loop
                position_loop()

    def position_loop():
        """Loops in Flight_Mode.POSITION until either the mode is switched or the target has been reached."""
        while self._mode == Flight_Mode.POSITON and not self._target_position_reached:
            # Build and publish the target position
            msg = PoseStamped(header = Header(stamp=rospy.Time.now()))
            msg.pose.x = self._target_position[0]
            msg.pose.y = self._target_position[1]
            msg.pose.z = self._target_position[2]

            # Break if within allowed distance of target
            def is_near(x, y):
                return abs(x - y) < 0.25

            if is_near(self._vehicle.get_position_x(), self._target_position[0]) and \
               is_near(self._vehicle.get_position_y(), self._target_position[1]) and \
               is_near(self._vehicle.get_position_z(), self._target_position[2]):
                break

        # Set mode to HOLD
        self._mode = Flight_Mode.HOLD
