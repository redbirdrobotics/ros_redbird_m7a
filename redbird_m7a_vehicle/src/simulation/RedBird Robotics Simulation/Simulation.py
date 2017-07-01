from Sim_Timer import Sim_Timer
from Target_Robot import Target_Robot
from Obstacle_Robot import Obstacle_Robots
import time
import threading

class Simulation(object):
    """description of class"""

    def __init__(self):
        self.Timer= Sim_Timer()

        robot = Target_Robot(0.5, 0.7, 1, 2, self.Timer)
        robot1 = Target_Robot(1.0, 6.7, 2, 1, self.Timer)
        #oRobot = Obstacle_Robots(TIMER)

        self.target_robot = []
        #self.obstacle_robot = []

        self.target_robot.append(robot)
        self.target_robot.append(robot1)
        #self.obstacle_robot.append(oRobot)

    def run(self):
        self.Timer.run()

        for arduino in self.target_robot:
            arduino.run()

        self.collision_thread()

        time.sleep(5)
        print(self.Timer.get_current_timer())

        self.Timer.pause() 

    def check_collision(self, robot_array):
        robot_array = [Target_Robot]
        while not (self.Timer.PAUSED.is_set()):
            Target_Robot.check_collisions(robot_array)

    def collision_thread(self):
        col_thread = threading.Thread(target = check_collision)

        try:
            col_thread.start()
            print("Collision thread has started")
        except:
            print("Could not start thread")