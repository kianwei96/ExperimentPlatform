from psychopy import event
from pylinkwrapper import connector
from psychopy.tools.monitorunittools import pix2deg
import os

class Eyelink():
    def __init__(self, win, edfname):
        self.tracker = connector.Connect(win, edfname)

    def get_position(self):
        x, y = self.tracker.get_gaze()
        elx = pix2deg(x - self.tracker.scenter[0],
                      self.tracker.win.monitor)
        ely = pix2deg(-(y - self.tracker.scenter[1]),
                      self.tracker.win.monitor)
        return elx, ely
        
    def start_recording(self):
        self.tracker.record_on(True)
        
    def stop_recording(self, datadir='data'):
        self.tracker.record_off()
        if not os.path.isdir(datadir):
            os.mkdir(datadir)
        self.tracker.end_experiment(os.path.join(os.getcwd(), datadir))
        
    def send_message(self, message):
        self.tracker.send_message(message)


class DummyTracker():
    def __init__(self, win):
        self.win = win
        self.mouse = event.Mouse()
    
    def get_position(self):
        return self.mouse.getPos()

    def start_recording(self):
        pass

    def stop_recording(self):
        pass

    def send_message(self, m):
        pass
