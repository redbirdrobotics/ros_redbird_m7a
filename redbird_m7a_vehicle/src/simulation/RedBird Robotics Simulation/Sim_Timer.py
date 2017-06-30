import threading
from time import sleep

class Sim_Timer(object):
    """description of class"""
    global PAUSED
    PAUSED = threading.Event()

    def __init__(self):
        self.counter = 0
        
    def run(self):
        Sim_timerthread = threading.Thread(target = self.update_time)
        
        PAUSED.clear()

        try:
            Sim_timerthread.start()
            print("Thread started for timer")

        except:
            print("Thread has failed")


    def update_time(self):
        global PAUSED

        print(PAUSED._flag)

        while not PAUSED.is_set():
            sleep(1)
            self.counter += 1
            print(self.get_current_timer())

    def get_current_timer(self):
        return self.counter

    def pause(self):
        global PAUSED
        PAUSED.set()

    def reset(self):
        self.counter = 0

    def get_pause(self):
        global PAUSED
        return PAUSED._flag