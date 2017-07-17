class Ground_Robot_Interface(object):
    """description of class"""
    global iterations
    iterations = 1

    def __init__(self, x, y, delta_x, delta_y, id, color):
        self._x = x
        self._y = y
        self._id = id
        self.color = color

        #radius of ground_robots
        self._radius = 0.34 #m

        #for obstacle robots travelling in a circle
        self._omega = 0

        self._deltaX = delta_x

        self._deltaY = delta_y

        #creating the flags for the timing, collisions, and border detection respectively
        self._timerUp = False
        self.collision = False
        self._boundary = False

        #creating variables for the timing
        self.start_timer = 0
        self.end_timer = 0
        self.deltaTime = 0
        
    def update_posX(self):
        pass

    def update_posY(self):
        pass
     
    def update_movement(self):
        pass

    def check_collisions(self, target_robot, obstacle_robots):
        pass

    def button_pushed(self, robot):
        pass

    def run(self, target_robots, obstacle_robots):
        pass
    
    def change_X_data(self, x):
        pass

    def change_Y_data(self, y):
        pass

    def change_VX_data(self, velocityX):
        pass

    def change_VY_data(self, velocityY):
        pass

    def check_error(self, x, y, velocityX, velocityY):
        pass
