from threading import Thread
from time import sleep

class Sim_Timer(object):
    """description of class"""
    global PAUSED
    PAUSED = False

    def __init__(self):
        self.counter = 0
        
    def run(self):

        Sim_timerthread = Thread(target = self.update_time)

        try:
            Sim_timerthread.start()
            print("Thread started for timer")

        except:
            print("Thread has failed")


    def update_time(self):
        global PAUSED

        while not PAUSED == True:
            sleep(1)
            self.counter += 1
            print(self.get_current_timer())

    def get_current_timer(self):
        return self.counter

    def pause(self):
        global PAUSED
        PAUSED = True

    def reset(self):
        self.counter = 0

    def get_pause(self):
        global PAUSED
        return PAUSED