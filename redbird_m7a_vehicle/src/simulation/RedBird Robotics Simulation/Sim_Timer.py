from threading import Thread
from time import sleep

class Sim_Timer(Thread, object):
    """description of class"""
    global PAUSED
    PAUSED = False

    def __init__(self):
        self.counter = 0
        
    def run(self):
        global PAUSED

        while not PAUSED == True:

            sleep(1.0)

            self.update_time()

            print(self.get_current_timer())


    def update_time(self):
        self.counter += 1

    def get_current_timer(self):
        return self.counter

    def pause(self):
        global PAUSED
        PAUSED = True

    def reset(self):
        self.counter = 0