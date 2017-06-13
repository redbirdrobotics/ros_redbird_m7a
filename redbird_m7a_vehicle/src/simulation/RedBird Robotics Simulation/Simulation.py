from Sim_Timer import Sim_Timer
from Target_Robot import Target_Robot

class Simulation(object):
    """description of class"""
    global TIMER
    def __init__(self):
        TIMER = Sim_Timer()
        robot = Target_Robot(0.5, 0.7, 1, 2, TIMER)

        self.target_robot = []

        self.target_robot.append(robot)

        for robot in self.target_robot:
            robot.run()

    def run(self):
        TIMER .run()