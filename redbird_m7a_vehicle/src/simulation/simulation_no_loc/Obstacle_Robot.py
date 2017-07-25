from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from math import sqrt, pow, cos, sin
from time import sleep
from threading import Thread

class Obstacle_Robot(Ground_Robot_Interface):
    def __init__(self, x, y, id, color, timer):
        #allowing for the timer to be synched with program
        self._timer = timer

        #getting class variables
        return super(Obstacle_Robot, self).__init__(x, y, id, color)

        #the radius of travel for the obstacle robot
        self.radius = 1.5 #m

        #finding the delta angle of travel
        self._omega = ( ( (0.33) / self.radius) * iterations)

    def update_posX(self):
        #finding the change in x of travel
        self._deltaX = cos(self._omega) * self.radius

        self._x = self._x + self._deltaX

    def update_posY(self):
        #finding the the change in y of travel
        self._deltaY = sin(self._omega) * self.radius

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
                print self.current_pos

                self.update_posX()
                self.update_posY()

                #making sure the angle is doubled
                self._omega = self._omega * 2

                self.deltaTime = self._timer.get_current_timer() - self.start_timer

                sleep(iterations)

            #resetting the timer
            self.deltaTime = 0

    def run(self):
        #creating the thread and making the target update movement
        self._ORThread = Thread(target = self.update_movement)

        try:
            self._ORThread.start()

            print "Thread Started"

        except:

            print "Thread could not start"

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def get_id(self):
        return self._id

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
