from Sim_Timer import Sim_Timer
from Target_Robot import Target_Robot
from Obstacle_Robot import Obstacle_Robots
import time

class Simulation(object):
    """description of class"""

    def __init__(self):
        self._timer = Sim_Timer()

        #oRobot = Obstacle_Robots(TIMER)

        #creating the main array of both the ground and obstacle robots
        self.target_robots = []
        self.obstacle_robots = []

        for robot in range(1, 10):
            self.target_robots.append(Target_Robot(0, 0, robot, 1, self._timer))

        for robot in range(11, 15):
            print("appending the obstacle robots")

            self.obstacle_robots.append(Obstacle_Robots(0,0, robot, 0, self._timer))

        print(self._timer.get_pause())

        #self.obstacle_robot = []

    def run(self):
        self._timer.run()

        #for arduino in self.target_robots:
        #    print("initing all of the ground robots")
        #    arduino.run(self.target_robot)

        for robot in self.obstacle_robots:
            print("initing the obstacle robots")
            robot.run()

        print("trying to sleep")

        time.sleep(5)
        print(self._timer.get_pause())

        print("trying to pause the sim")

        self._timer.pause()

    def get_robots(self):
        return self.target_robot