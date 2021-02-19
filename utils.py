import serial
import warnings
from psychopy import core

class Reward:
    def __init__(self, serialpath=None, reward_duration=0.5):
        self.dummy = False
        if serialpath is None:
            self.port = None
        else:
            try:
                self.port = serial.Serial(serialpath)
                self.port.setRTS(0) #start in closed mode
            except:
                self.port = None
                self.dummy = True
                warnings.warn("Could not open serial port at %s" %(serialpath, ))

        self.duration = reward_duration
        self.total_duration = 0.0

    def open(self):
        if self.port is not None:
            self.port.setRTS(1)
        else:
            print ("Reward port open")

    def close(self):
        if self.port is not None:
            self.port.setRTS(0)
        else:
            print ("Reward port closed")

    def deliver(self, duration=None):
        if duration is None:
            duration = self.duration
        self.open()
        core.wait(duration)
        self.close()


def calc_flowrate(x, a=0.00014933173223849661, b=1.6854748802635457):
    """
    :param x: height difference between water surface and juicer mouth piece
    :param a: coefficient
    :param b: coefficient
    :return: a*x**b
    """
    return a*x**b