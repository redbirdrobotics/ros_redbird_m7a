from Sim_Timer import Sim_Timer
from Target_Robot import Target_Robot
from Obstacle_Robot import Obstacle_Robots
import time

class Simulation(object):
    """description of class"""
    global Timer
    Timer = Sim_Timer()

    def __init__(self):
        global Timer
        self.TIMER = Sim_Timer()

        robot = Target_Robot(0.5, 0.7, 1, 2, self.TIMER)
        robot1 = Target_Robot(1.0, 6.7, 2, 1, Timer)
        #oRobot = Obstacle_Robots(TIMER)

        self.target_robot = []
        #self.obstacle_robot = []

        self.target_robot.append(robot)
        self.target_robot.append(robot1)
        #self.obstacle_robot.append(oRobot)

    def run(self):
        for arduino in self.target_robot:
            arduino.run()

        self.TIMER.run()

        time.sleep(5.5)
        print(self.TIMER.get_current_timer())

        self.TIMER.pause() 