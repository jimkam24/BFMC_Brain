import threading
import base64
import time
import serial
import numpy as np

from src.gps.threads.Gps import Gps

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    Pos,
    Location,
    ImuData
)
from src.templates.threadwithstop import ThreadWithStop

class threadGps(ThreadWithStop):

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadGps, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.gps = Gps()

        # Configure the Serial Port
        self.ser = serial.Serial(
            port="/dev/ttyACM0",\
            baudrate=115200,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=None)
        print("connected to: " + self.ser.portstr)



    # =============================== STOP ================================================
    def stop(self):
        super(threadGps, self).stop()

    # =============================== START ===============================================
    def start(self):
        super(threadGps, self).start()

    # =============================== CONFIG ==============================================
    # def Configs(self):
    #     """Callback function for receiving configs on the pipe."""

    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True"""
        while self._running:
            coords = self.gps.update(self.ser)
            if coords is not None:
                self.queuesList[Location.Queue.value].put(      #send back coordinates
                        {
                            "Owner": Location.Owner.value,
                            "msgID": Location.msgID.value,
                            "msgType": Location.msgType.value,
                            "msgValue": coords
                        }
                    )






            