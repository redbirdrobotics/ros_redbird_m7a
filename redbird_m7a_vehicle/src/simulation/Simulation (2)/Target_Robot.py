from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer
from time import sleep
from threading import Thread

class Target_Robot(Ground_Robot_Interface, object):
    """description of class"""

    def __init__(self, x, y, id, color, timer): 
        self._timer = timer 

        return super().__init__(x, y, id, color)

    def update_posX(self):
        changeX = self._deltaX * iterations
        self._x = changeX + self._x
        return self._x

    def update_posY(self):
        changeY = self._deltaY * iterations
        self._y = changeY + self._y
        return self._y
     
    def update_movement(self):
        while not (self._timer._PAUSED.is_set()):
            self.current_pos = (self._x, self._y , self._id)
            print(self.current_pos)
            print(self._timer.get_pause())

            if(self.collision == True):
                self._deltaX = self._deltaX * -1

                self.deltaY = self.deltay * -1 

                sleep(iterations)

            else:
                self.update_posX()
                self.update_posY()

                sleep(iterations)
        #super().set_coordinates(self.x, self.y) 

    def check_collisions(self, target_robot):
        min_num = 0
        max_num = len(target_robot)

        target_robot = [Target_Robot]

        while min_num < max_num:
            for robot in range(1, len(target_robot)):

                if(target_robot[robot].boundary):
                    del target_robot[robot]

                    break

                if (target_robot[robot]._x >= 10 and target_robot[robot]._y >= 10):
                    target_robot[robot].boundary = True

                else:
                    dXX = target_robot[min_num]._x - target_robot[robot]._x
                    dYY = target_robot[min_num]._y - target_robot[robot]._y

                    dCC = sqrt((pow(dXX, 2) + pow(dYY, 2)))

                    if dCC <= 2*radius :
                        target_robot[min_num].button_pushed(target_robot[robot])

                        self.collision = True

            min_num = min_num + 1

    def button_pushed(self, robot):
         robot = Target_Robot
         
         vector_i = self._x - robot._x
         vector_j = self._y - robot._y

         theta = tan(vector_j / vector_i)

         min_theta = self.theta - 70
         max_theta = self.theta + 70

         if(theta >= min_theta and theta <= max_theta):
             self.button_pushed = True

    def run(self, target_robots):
        self._distanceThread = Thread(target = self.update_movement)

        self._collision_thread = Thread(target = self.check_collisions, args = (target_robots,))

        start_time = self._timer.get_current_timer()
        current_time = 0

        try:
            self._distanceThread.start()
            self._collision_thread.start()

            print("Thread has started!")
        except:
            print("Thread failed")

        #while not PAUSED == True and current_time < 20:
        #    self.update_movement()

        #    current_time = self.Timer.get_current_timer() - start_time

        #    sleep(1.0)
    
    def change_X_data(self, x):
        self._x = x

    def change_Y_data(self, y):
        self._y = y

    def change_VX_data(self, velocityX, velocityY):
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