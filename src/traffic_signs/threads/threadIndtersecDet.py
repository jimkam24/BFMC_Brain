import cv2
import threading
import base64
import time
import numpy as np


import src.traffic_signs.threads.inter_det_new as inter 

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    Estimate,
    InterDistance,
    serialCamera
)
from src.templates.threadwithstop import ThreadWithStop

class threadInterDet(ThreadWithStop):

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadInterDet, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        pipeRecvEst, pipeSendEst = Pipe()
        self.pipeRecvEst = pipeRecvEst
        self.pipeSendEst = pipeSendEst
        pipeRecvImg, pipeSendImg = Pipe()
        self.pipeRecvImg = pipeRecvImg
        self.pipeSendImg = pipeSendImg
        self.estimate = False
        self.subscribe()
        self.pipeRecvEst.send("ready")   #send ready flag through pipe 

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Estimate.Owner.value,
                "msgID": Estimate.msgID.value,
                "To": {"receiver": "threadInterDet", "pipe": self.pipeSendEst},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": serialCamera.Owner.value,
                "msgID": serialCamera.msgID.value,
                "To": {"receiver": "threadIntersecDet", "pipe": self.pipeSendImg},
            }
        )

    '''
    Maybe add this function later, if we want to send back an
    acknolegdment message, saying that we recieved the 'record' flag
    '''
    # def Queue_Sending(self):
    #     """Callback function for recording flag."""
    #     self.queuesList[Recording.Queue.value].put(
    #         {
    #             "Owner": Recording.Owner.value,
    #             "msgID": Recording.msgID.value,
    #             "msgType": Recording.msgType.value,
    #             "msgValue": self.recording,
    #         }
    #     )
    #     threading.Timer(1, self.Queue_Sending).start()

    # =============================== STOP ================================================
    def stop(self):
        super(threadInterDet, self).stop()

    # =============================== START ===============================================
    def start(self):
        super(threadInterDet, self).start()

    # =============================== CONFIG ==============================================
    # def Configs(self):
    #     """Callback function for receiving configs on the pipe."""

    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True"""

        while self._running:
            
            if self.pipeRecvEst.poll():

                self.estimate = self.pipeRecvEst.recv()
                self.estimate = self.estimate['value']
            
            if self.estimate:#if there has been a valid request for intersection distance estimation
                self.pipeRecvImg.send("ready")                      #ready for frame

                frame = self.pipeRecvImg.recv()                     #recieve image
                image_data = base64.b64decode(frame['value'])       #decode frame
                frame = np.frombuffer(image_data, dtype=np.uint8)
                frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                results = inter.inter_det(frame)        #detect intersection
                # print(results)
                if results is not None:
                    frame, start, end = results
                    coordinates = inter.calculate_distance(start)       #calculate distance
                    distance = coordinates[2]                           #keep Z-value
                    dist_from_front = 0.17
                    distance = distance - dist_from_front
                    # print(distance)
                    if distance >= 0.4 and distance<=0.7:

                        self.queuesList[InterDistance.Queue.value].put(     #send back calculated distance
                            {
                                "Owner": InterDistance.Owner.value,
                                "msgID": InterDistance.msgID.value,
                                "msgType": InterDistance.msgType.value,
                                "msgValue": distance
                            }
                        )
                        self.estimate = False                               #reset flag value
                        self.pipeRecvEst.send("ready")                          #send ready flag through pipe 
                
            

    
    

