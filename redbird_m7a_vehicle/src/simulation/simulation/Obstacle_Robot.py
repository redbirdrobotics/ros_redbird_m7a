from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from math import sqrt, pow, cos, sin
from time import sleep
from threading import Thread

class Obstacle_Robot(Ground_Robot_Interface, object):

    def __init__(self, x, y, delta_x, delta_y, id, color, timer):
        #allowing for the timer to be synched with program
        self._timer = timer

        #getting class variables
        return super().__init__(x, y, delta_x, delta_y, id, color)

        #the radius of travel for the obstacle robot
        self.radius = 1.5 #m
        
        #finding the delta angle of travel
        self._omega = ( ( (sqrt( ( pow(self._deltaX, 2) + pow(self._deltaY, 2) ) ) ) / self.radius) * iterations)

        #finding the change in x of travel
        self._deltaX = cos(self._omega)

        #finding the the change in y of travel
        self._deltaY = sin(self._omega)

    def update_posX(self):
        self._x = self._x + self._deltaX

    def update_posY(self):
        self._y = self._y + self._deltaY

    def update_movement(self):
        #only allwing the code to work if the timer is not paused
        while not (self._timer._PAUSED.is_set()):
            #as long as the change in time is no less than 20 than the code will work
            self.start_timer = self._timer.get_current_timer()

            while self.deltaTime <= 20:
                #starting the timer

                #setting the currrent position and printing
                self.current_pos = (self._x, self._y , self._id)
                print(self.current_pos)

                self.update_posX()
                self.update_posY()

                #making sure the angle is doubled
                """self._omega = self._omega * 2

                self._deltaX = cos(self._omega)

                self._deltaY = sin(self._omega)"""

                self.deltaTime = self._timer.get_current_timer() - self.start_timer

                print("delta time for obstacle robot is " + str(self.deltaTime))

                sleep(iterations)
            
            #resetting the timer
            self.deltaTime = 0

    def run(self):
        #creating the thread and making the target update movement
        self._ORThread = Thread(target = self.update_movement)
        try:
            self._ORThread.start()
            print("Thread Started")
        except:
            print("Thread could not start")

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def get_id(self):
        return self._id