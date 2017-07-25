from Sim_Timer import Sim_Timer
from Target_Robot import Target_Robot
from Obstacle_Robot import Obstacle_Robot
from threading import Thread
from Ground_RobotInterface import iterations
from math import sqrt, pow, tan
import rospy

class Simulation(object):
    """description of class"""

    def __init__(self):
        self._timer = Sim_Timer()

        #creating the main array of both the ground and obstacle robots with arbitrary values
        self.Rtarget_robots = []
        self.Gtarget_robots = []
        self.Wobstacle_robots = []
        self.target_robots = []

        #Filling the target robot array with arbitrary values
        for robot in range(1, 5):
            self.Rtarget_robots.append(Target_Robot(0, 0, robot, 0, self._timer))

        for robot in range(1, 5):
            self.Gtarget_robots.append(Target_Robot(0, 0, robot, 1, self._timer))

        self.target_robots.extend(self.Rtarget_robots)
        self.target_robots.extend(self.Gtarget_robots)

        #Filling the obstacle robot array
        for robot in range(1, 5):
            self.Wobstacle_robots.append(Obstacle_Robot(0, 0, robot, 2, self._timer))

    def run(self):
        #Running the timer (controls all threads)
        self._timer.run()

        #Runs all threads in the target robot array
        for arduino in self.target_robots:
            print 'initing all of the ground robots'
            arduino.run(self.Wobstacle_robots)

        #Runs all threads in the obstacle robot array
        for robot in self.Wobstacle_robots:
            print 'initing the obstacle robots'
            robot.run()

        self.threading()

    def get_G_Target_robots(self):
        return self.Gtarget_robots

    def get_R_Target_robots(self):
        return self.Rtarget_robots

    def get_Obstacle_Robots(self):
        return self.Wobstacle_robots

    def check_collision(self):
        #Only allowing for the PAUSED flag to be false
        while not self._timer._PAUSED.is_set():

            min_num = 0
            max_num = len(self.target_robots)

            while min_num < max_num :

                #looping through the array that is being input to the function
                for robot in range(1, max_num):

                    #as long as the two id are not equal
                    if (self.target_robots[min_num]._color == self.target_robots[robot]._color):

                        if not (self.target_robots[min_num]._id == self.target_robots[robot]._id):
                            self.check_calculations(min_num, robot)

                    else:
                        self.check_calculations(min_num, robot)

                min_num +=1
                rospy.sleep(iterations)

    def check_calculations(self, min_num, robot):
        #as long as the boundary flag is raised
        if(self.target_robots[min_num]._boundary):

            #stopping and deleting the thread
            self.target_robots[min_num]._distanceThread._stop()

            self.target_robots[min_num]._distanceThread._delete()

        #testing the if the robot has exited the boundary
        elif(self.target_robots[min_num]._x >= 10 and self.target_robots[min_num]._y >= 10):
            self.target_robots[min_num]._boundary = True

        else:
            #Determining the distance from robot to robot
            dXX = self.target_robots[robot]._x - self.target_robots[min_num]._x
            dYY = self.target_robots[robot]._y - self.target_robots[min_num]._y

            dCC = sqrt((pow(dXX, 2) + pow(dYY, 2)))

            max_distance = (self.target_robots[min_num]._radius + self.target_robots[robot]._radius)

            #if the distance from center to center is less than the sum of the two radii
            if dCC <= max_distance:

                self.button_pushed(self.target_robots[min_num], self.target_robots[robot])

    def button_pushed(self, robot, robot1):
        #arbitrary definition of the two inputs
        robot = Target_Robot
        robot1 = Target_Robot

        #finding the distance vectors between each of the robots
        vector_i = robot._x - robot1._x
        vector_j = robot._y - robot1._y

        if (vector_i == 0):
            theta = 90
        else:
            theta = tan(vector_j / vector_i)

        #Calculating the angle of velocity vector and bounds checking
        if(robot._deltaX == 0):

            if(robot._deltaY < 0):
                v_theta = 180

            v_theta = 90

        else:
            v_theta = tan(robot._deltaY / robot._deltaX)

        #Finding the relative angle of where the button is pushed
        min_theta = robot.get_theta() - 70
        max_theta = robot.get_theta() + 70

        #Calculating the angle of velocity vector with respect to the other robot
        #this is necessary because now the Simulation is doing collision detection

        min2_theta = robot1.get_theta() - 70
        max2_theta = robot1.get_theta() + 70

        if(theta >= min_theta and theta <= max_theta):
            robot.collision = True

        if(theta >= min2_theta and theta <= max2_theta):
            robot1.collision = True

    def threading(self):
        #starting and initializing the thread

        self._collision_thread = Thread(target = self.check_collision)

        try:
            self._collision_thread.start()

            print 'Thread started'

        except:
            print 'Thread not started'
