from threading import Thread
import time

class Sim_Timer(object):
    """description of class"""
    PAUSED = False

    def __init__(self):
        self.counter = 0
        
    def run(self):
        while not PAUSED:
            time.sleep(1.0)
            self.update_time()

    def update_time(self):
        self.counter += 1
        print(self.counter)
        return self.counter

    def get_current_timer(self):
        return self.counter

    def stop(self):
        PAUSED = True

    def reset(self):
        self.counter = 0