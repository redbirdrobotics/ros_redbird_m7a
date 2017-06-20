from Ground_RobotInterface import Ground_Robot_Interface, iterations
from Sim_Timer import Sim_Timer, PAUSED
from time import sleep
from threading import Thread

class Target_Robot(Ground_Robot_Interface, object):
    """description of class"""
    

    def __init__(self, x, y, id, color, timer):
        timer = Sim_Timer()  

        self.Timer = timer

        return super().__init__(x, y, id, color)

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

        #super().set_coordinates(self.x, self.y) 

    def check_collisions(self, target_robot):
        min_num = 0
        max_num = len(target_robot)

        target_robot = [Target_Robot]

        while min_num < len(target_robot):
            for robot in range(1, len(target_robot)):
                dXX = target_robot[min_num].x - target_robot[robot].x
                dYY = target_robot[min_num].y - target_robot[robot].y

                dCC = sqrt((pow(dXX, 2) + pow(dYY, 2)))

                if dCC <= 2*radius :
                    target_robot[min_num].button_pushed(target_robot[robot])
                    self.collision = True

            min_num = min_num + 1

    def button_pushed(self, robot):
         robot = Target_Robot
         
         vector_i = self.x - robot.x
         vector_j = self.y - robot.y

         theta = tan(vector_j / vector_i)

         min_theta = self.theta - 70
         max_theta = self.theta + 70

         if(theta >= min_theta and theta <= max_theta):
             self.button_pushed = True

    def run(self):
        global PAUSED
        start_time = self.Timer.get_current_timer()
        current_time = 0

        while not PAUSED == True and current_time < 20:
            self.update_movement()

            current_time = self.Timer.get_current_timer() - start_time

            sleep(1.0)


    def error(self, current_position, velocity_vector, angle):
        errorVX = self.deltaX - velocity_vector[0]
        errorVY = self.detlaY - velocity_vector[1]

        if(errorVX >= 0.01):
            self.deltaX = velocity_vector[0]
            if(errorVY >= 0.01):
                self.deltaY = velocity_vector[1]
                

    """def confidenceInterval(self, XY):
        counter = 0
        sum = 0

        for robot in self.XYID:
            sumX = sumX + robot[0]
            sumY = sumY + robot[1]
            counter += 1

        meanX = sumX/counter
        meanY = sumY/counter"""
    
    def change_X_data(self, x):
        self.x = x

    def change_Y_data(self, y):
        self.y = y

    def change_VX_data(self, velocityX, velocityY):
        self.deltaX = velocityX

    def change_VY_data(self, velocityY):
        self.deltaY = velocityY

    def check_error(self, x, y, velocityX, velocityY):
        pErrorX = ((self.x - x) /  x)
        
        if not (pErrorX <= 0.001):
            self.change_X_data(x)

        pErrorY = ((self.y - y) /  y) 

        if not (pErrorY <= 0.001):
            self.change_Y_data(y)

        pErrorVX = ((self.deltaX - velocityX) / velocityX)

        if not (pErrorVX<= 0.001):
            self.change_VX_data(velocityX)

        pErrorVY = ((self.deltaY - velocityY) / velocityY)

        if not (pErrorVY<= 0.001):
            self.change_VY_data(velocityY)