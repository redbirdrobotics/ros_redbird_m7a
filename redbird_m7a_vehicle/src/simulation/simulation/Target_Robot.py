from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from Obstacle_Robot import Obstacle_Robot
from time import sleep
from threading import Thread
from math import fabs

class Target_Robot(Ground_Robot_Interface, object):
    """description of class"""

    def __init__(self, x, y, id, color, timer): 
        self._timer = timer

        return super().__init__(x, y, id, color,)

    def update_posX(self):
        changeX = self._deltaX * iterations
        self._x = changeX + self._x
        return self._x

    def update_posY(self):
        changeY = self._deltaY * iterations
        self._y = changeY + self._y
        return self._y
     
    def update_movement(self, obstacle_robots, target_robots):
        while not self._timer._PAUSED.is_set():

            print(self.current_pos)
            print(self._timer.get_pause())

            if(self.collision == True):
                self._deltaX = self._deltaX * -1

                self.deltaY = self.deltay * -1 

                sleep(iterations)

            else:
                self._current_pos = (self.update_posX(), self.update_posY(), self._id)

                sleep(iterations)

            self.check_collisions(target_robots, obstacle_robots)
        #super().set_coordinates(self.x, self.y) 

    def check_collisions(self, target_robots, obstacle_robots):
        min_num = 0
        max_num = len(target_robots)

        target_robots = [Target_Robot]
        obstacle_robots = [Obstacle_Robot]

        while min_num < max_num:

            for robot in range(1, max_num):

                if(target_robots[robot].boundary):
                    del target_robots[robot]

                    break

                if (target_robots[robot]._x >= 10 and target_robots[robot]._y >= 10):
                    target_robots[robot].boundary = True

                else:
                    dXX = fabs(target_robots[min_num]._x - target_robots[robot]._x)
                    dYY = fabs(target_robots[min_num]._y - target_robots[robot]._y)

                    dCC = sqrt((pow(dXX, 2) + pow(dYY, 2)))

                    if dCC <= 2*self._radius:
                        target_robots[min_num].button_pushed(target_robots[robot])

                        self.collision = True

            min_num = min_num + 1

        for robot in target_robots:

            for oRobot in obstacle_robots:

                dXX = fabs(oRobot._x - target_robots[min_num]._x)
                dYY = fabs(oRobot._y - target_robots[min_num]._y)

                dCC = sqrt((pow(dXX, 2)) + pow(dYY, 2))

                if dCC <= 2 * self._radius:

                    target_robots[min_num].oButton_pushed(oRobot)

    def button_pushed(self, robot):
         robot = Target_Robot
         
         vector_i = self._x - robot._x
         vector_j = self._y - robot._y

         theta = tan(vector_j / vector_i)

         min_theta = self.theta - 70
         max_theta = self.theta + 70

         if(theta >= min_theta and theta <= max_theta):
             self.button_pushed = True

    def oButton_pushed(self, robot):
         robot = Obstacle_Robot
         
         vector_i = self._x - robot._x
         vector_j = self._y - robot._y

         theta = tan(vector_j / vector_i)

         min_theta = self.theta - 70
         max_theta = self.theta + 70

         if(theta >= min_theta and theta <= max_theta):
             self.button_pushed = True

    def run(self, target_robots, obstacle_robots):
        self._target_robot = Thread(target = self.update_movement(), args = (target_robots, obstacle_robots,))

        try:
            self._target_robot.start()

            print("Thread has started")

        except:
            print("Thread could not start")

    def update_posX(self):
        changeX = self._deltaX * iterations
        self._x = changeX + self._x
        return self._x

    def update_posY(self):
        changeY = self._deltaY * iterations
        self._y = changeY + self._y
        return self._y

    def update_movement(self):
        self.current_pos = (self._x, self._y , self._id)
        print(self.current_pos)

        if(self.collision == True):
            self.deltaX = self._deltaX * -1

            self._deltaY = self.deltay * -1 

            sleep(1)

        else:
            self.update_posX()
            self.update_posY()

    def change_X_data(self, x):
        self._x = x

    def change_Y_data(self, y):
        self._y = y

    def change_VX_data(self, velocityX):
        self._deltaX = velocityX

    def change_VY_data(self, velocityY):
        self._deltaY = velocityY

    def check_error(self, x, y, velocityX, velocityY):
        pErrorX = ((self._x - x) /  x)
        
        if not (pErrorX <= 0.001):
            self.change_X_data(x)

        pErrorY = ((self._y - y) /  y) 

        if not (pErrorY <= 0.001):
            self.change_Y_data(y)

        pErrorVX = ((self._deltaX - velocityX) / velocityX)

        if not (pErrorVX <= 0.001):
            self.change_VX_data(velocityX)

        pErrorVY = ((self._deltaY - velocityY) / velocityY)

        if not (pErrorVY<= 0.001):
            self.change_VY_data(velocityY)