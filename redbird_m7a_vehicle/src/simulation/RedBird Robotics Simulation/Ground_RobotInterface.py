from Sim_Timer import Sim_Timer
from threading import Thread
from random import randint
from math import tan, pow, sqrt

class Ground_Robot_Interface(Thread, object):
    """description of class"""
    global iterations
    iterations = 60

    def __init__(self, x, y, ID, color):
        self.x = x
        self.y = y
        self.ID = ID
        self.color = color

        self.deltaX = randint(-33, 33) / 100
        self.deltay = sqrt(((pow(0.33, 2)) - (pow(self.deltaX, 2))))
        self.angle = tan((self.deltay / self.deltaX))

        self.thread_cancelled = False
        self.collision = False

        self.pError = 0.0 
        
    def get_coordinates(self):
        return (self.x, self.y)

    def get_id(self):
        return self.ID

    def set_coordinates(self, x, y):
        self.x = x
        self.y = y

    def get_angle(self):
        pass

    def run(self):
        pass

    def cancel(self):
        pass

    def update_movement(self):
        pass

    def check_collisions(self, ):
        pass