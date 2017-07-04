from Ground_RobotInterface import Ground_Robot_Interface
from Sim_Timer import Sim_Timer
from math import pow, sqrt, cos, sin

class Obstacle_Robots(Ground_Robot_Interface, object):
    radius = 1.0 #m
    global orID
    def __init__(self, timer):
        self.timer = timer = Sim_Timer()
        return super.__init__(self)

        velocity = sqrt((pow(self.deltaX,2 ) + pow(self.deltay, 2)))

        omega = velocity / radius

        deltaO = omega * radius

        self.deltaOX = cos(omega)
        self.deltaOY = sin(omega)

        orID = 11

    def update_movement(self):
        self.x = self.x + self.deltaOX
        self.y = self.y + self.deltaOY

        self.current_pos = (orID, self.x, self.y) 