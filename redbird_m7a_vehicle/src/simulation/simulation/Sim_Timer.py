from threading import Thread, Event
from time import sleep

class Sim_Timer(object):
    """description of class"""

    def __init__(self):
        self.counter = 0
        self._PAUSED = Event()

    def run(self):
        #Creating the timing thread
        Sim_timerthread = Thread(target = self.update_time)

        #setting the flag to false
        self._PAUSED.clear()

        try:
            #starting the timer thread
            Sim_timerthread.start()
            print("Thread started for timer")

        except:
            print("Thread has failed")

    def update_time(self):

        #only allowing for the flag to be false 
        while not self._PAUSED.is_set():

            #stopping for 1 second
            sleep(1)

            #incrementing the timer by one and printing
            self.counter += 1
            print(self.get_current_timer())

    def get_current_timer(self):
        return self.counter

    def pause(self):
        #setting the flag to true
        self._PAUSED.set()

    def reset(self):
        self.counter = 0

    def get_pause(self):
        return self._PAUSED._flag

    def resume(self):
        self._PAUSED.clear()

    """def quit(self):
        self.pause()
        self._quit.set()"""