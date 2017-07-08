from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from math import sqrt, pow, cos, sin
from time import sleep
from threading import Thread

class Obstacle_Robots(Ground_Robot_Interface, object):

    def __init__(self, x, y, id, color, timer):
        self.radius = 1.5 #m

        self._timer = timer

        return super().__init__(x, y, id, color)
        
        self._omega = ( ( (sqrt( ( pow(self._deltaX, 2) + pow(self._deltaY, 2) ) ) ) / self.radius) * iterations)

        self._deltaX = cos(self._omega)

        self._deltaY = sin(self._omega)

    def update_movement(self):
        while not (self._timer._PAUSED.is_set()):
            self.current_pos = (self._x, self._y , self._id)
            print(self.current_pos)

            self._x = self._x + self._deltaX
            self._y = self._y + self._deltaY

            #self._omega = self._omega * 2

            self._deltaX = cos(self._omega)

            self._deltaY = sin(self._omega)

            sleep(iterations)

    def run(self):
        self._OR_Thread = Thread(target = self.update_movement())

        try:
            self._OR_Thread.start()
            print("Obstacle Robot thread started")

        except:
            print("Could not start thread")