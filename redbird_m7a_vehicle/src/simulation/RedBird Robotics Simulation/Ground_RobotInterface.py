from threading import Thread
from random import randint
from math import tan, pow, sqrt
from time import sleep

class Ground_Robot_Interface(Thread, object):
    """description of class"""
    global iterations
    iterations = 1 

    def __init__(self, x, y, deltaX, deltaY, ID, color):
        self.x = x
        self.y = y
        self.ID = ID
        self.color = color
        self.radius = 1.0

        self.deltaX = deltaX

        self.deltaY = deltaY

        #self.deltaX = ((randint(-33, 33)) / 100 )

        #self.deltaY = sqrt(((pow(0.33, 2)) - (pow(self.deltaX, 2))))
        
        if(self.deltaX != 0):
            self.angle = tan((self.deltaY / self.deltaX))
        else:
            self.angle = 90

        self.thread_cancelled = False
        self.collision = False

        self.pError = 0.0 
        self.pErrorV = 0.0
        
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

    def update_posX(self):
        changeX = self.deltaX * iterations
        self.x = changeX + self.x
        return self.x

    def update_posY(self):
        changeY = self.deltaY * iterations
        self.y = changeY + self.y
        return self.y


    def update_movement(self):
        self.current_pos = (self.x, self.y , self.ID)
        print(self.current_pos)

        if(self.collision == True):
            self.deltaX = self.deltaX * -1

            self.deltaY = self.deltay * -1 

            sleep(1)

        else:
            self.update_posX()
            self.update_posY()

    def check_collisions(self):
        pass