from Sim_Timer import Sim_Timer
from Target_Robot import Target_Robot
from Obstacle_Robot import Obstacle_Robots
from time import sleep

class Simulation(object):
    """description of class"""

    def __init__(self):
        self.TIMER = Sim_Timer()
        robot = Target_Robot(0.5, 0.7, 1, 2, self.TIMER)
        #oRobot = Obstacle_Robots(TIMER)

        self.target_robot = []
        #self.obstacle_robot = []

        self.target_robot.append(robot)
        #self.obstacle_robot.append(oRobot)

        for robot in self.target_robot:
            robot.run()

    def run(self):
        self.TIMER.run()

        sleep(5)

        self.TIMER.pause() 