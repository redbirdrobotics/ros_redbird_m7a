from threading import Thread, Event
from time import sleep

class Sim_Timer(object):
    """description of class"""

    def __init__(self):
        self.counter = 0
        self._PAUSED = Event()
        
    def run(self):

        Sim_timerthread = Thread(target = self.update_time)
        self._PAUSED.clear()

        try:
            Sim_timerthread.start()
            print("Thread started for timer")

        except:
            print("Thread has failed")


    def update_time(self):

        while not self._PAUSED.is_set():
            sleep(1)
            self.counter += 1
            print(self.get_current_timer())

    def get_current_timer(self):
        return self.counter

    def pause(self):
        self._PAUSED.set()

    def reset(self):
        self.counter = 0

    def get_pause(self):
        return self._PAUSED._flag