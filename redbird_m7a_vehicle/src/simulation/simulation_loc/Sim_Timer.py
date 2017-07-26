from threading import Thread, Event
import rospy

class Sim_Timer(object):
    """description of class"""

    def __init__(self):
        self.counter = 0

        #creating two events to control the program
        self._PAUSED = Event()
        self._quit = Event()

    def run(self):
        #Creating the timing thread
        Sim_timerthread = Thread(target = self.update_time)

        #setting the flag to false
        self._PAUSED.clear()

        self._quit.clear()

        try:
            #starting the timer thread
            Sim_timerthread.start()

        except:
            rospy.loginfo("Timing thread has failed")

    def update_time(self):

        while not rospy.is_shutdown():
            #allows the program to be paused and resumed
            while not (self._quit.is_set()):

                #only allowing for the flag to be false 
                while not self._PAUSED.is_set():

                    #stopping for 1 second
                    rospy.sleep(1)

                    #incrementing the timer by one and printing
                    self.counter += 1

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

    def quit(self):
        self._PAUSED.set()

        self._quit.set()