from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from math import pow, sqrt, cos, sin
import threading

class Obstacle_Robots(Ground_Robot_Interface, object):

    def __init__(self, x, y, id, color, timer):
        self.timer = timer = Sim_Timer()

        self.radius = 2

        return super.__init__(x, y, id, color)

        deltaOmega = (sqrt(pow(self.deltaX, 2) + pow(self.deltaY, 2))) / radius

        omega = deltaOmega * iterations

        self.deltaX = cos(omega)
        self.detlaY = sin(omega)

    def update_movement(self):
        while not (self.timer.PAUSED.is_set()):
            self.x = self.x + self.deltaX
            self.y = self.y + self.deltaY

            self.current_pos = (orID, self.x, self.y)

    def run(self):
        oRobotThread = threading.Thread(target = update_movement)

        try:
            oRobotThread.start()
            print("Obstacle Robots started")

        except:
            print("Could not start thread")