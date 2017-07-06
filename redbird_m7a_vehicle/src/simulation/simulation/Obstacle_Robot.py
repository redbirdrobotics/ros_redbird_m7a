from Ground_RobotInterface import Ground_Robot_Interface
from Sim_Timer import Sim_Timer
from math import pow, sqrt, cos, sin
from threading import Thread

class Obstacle_Robots(Ground_Robot_Interface, object):

    def __init__(self, x, y, color, id, timer):
        self._timer = timer

        return super().__init__(x, y, id, color)
        
        
        
    def update_movement(self):
        self.x = self.x + self._deltaX
        self.y = self.y + self._deltaY

        self.current_pos = (orID, self.x, self.y) 

    def run(self):
        self._OR_Thread = Thread(target = self.update_movement())

        try:
            self._OR_Thread.start()
            print("Obstacle Robot")

        except:
            print("Could not start thread")