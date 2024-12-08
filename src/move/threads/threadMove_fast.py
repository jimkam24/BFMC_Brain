# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
import threading
import time
import cv2
import numpy as np
import base64
import logging
from collections import deque

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
ImuData, 
serialCamera,
EngineRun,
TrafficSign,
BatteryLvl,
mainCamera,
MoveConfig,
Estimate,
InterDistance,
Infrared
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
import src.move.threads.movements.lane_following as lf
import src.move.threads.movements.lane_following_old as lf_old
from src.move.threads.movements.intersection import round90, intersection_navigation

class threadMove_fast(ThreadWithStop):
    """Thread which will handle the decision making.\n
    Args:
        pipeRecv: maybe
        pipeSend: maybe
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadMove_fast, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.recording = False
        self.pipes = list()
        pipeRecvstart, pipeSendstart = Pipe()
        self.pipeRecvstart = pipeRecvstart
        self.pipeSendstart = pipeSendstart
        self.pipeRecvstart.send("ready")
        pipeRecvcamera_lf, pipeSendcamera_lf = Pipe()
        pipeRecvconfig, pipeSendconfig = Pipe()
        self.pipeRecvconfig = pipeRecvconfig
        self.pipeSendconfig = pipeSendconfig
        pipeIMUrecv, pipeIMUsend = Pipe()
        self.pipeIMUrecv = pipeIMUrecv
        self.pipeIMUsend = pipeIMUsend
        self.pipeRecvconfig.send("ready")
        pipeRecvInterDet, pipeSendInterDet = Pipe()
        self.pipeRecvInterDet = pipeRecvInterDet
        self.pipeSendInterDet = pipeSendInterDet
        # self.pipeRecvInterDet.send("ready")
        pipeRecvinf, pipeSendinf = Pipe()
        self.pipeRecvinf = pipeRecvinf
        self.pipeSendinf = pipeSendinf
        self.pipeRecvinf.send("ready")
        self.pipes.append(self.pipeRecvinf)
        self.pipes.append(self.pipeRecvInterDet)

        self.K = 0.11
        self.speed = 16#15 is default
        self.engine = False
        self.autonomous = True

        self.pipeIMUrecv = pipeIMUrecv
        # self.pipes.append(self.pipeIMUrecv)
        self.pipeIMUsend = pipeIMUsend       
        self.pipeRecvcamera_lf = pipeRecvcamera_lf
        self.pipes.append(self.pipeRecvcamera_lf)
        self.pipeSendcamera_lf = pipeSendcamera_lf
        self.subscribe()
        # for pipe in self.pipes:
        #     pipe.send("ready")

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": serialCamera.Owner.value,
                "msgID": serialCamera.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendcamera_lf},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": EngineRun.Owner.value,
                "msgID": EngineRun.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendstart},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": ImuData.Owner.value,
                "msgID": ImuData.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeIMUsend},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": MoveConfig.Owner.value,
                "msgID": MoveConfig.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendconfig}
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": InterDistance.Owner.value,
                "msgID": InterDistance.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendInterDet},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Infrared.Owner.value,
                "msgID": Infrared.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendinf},
            }
        )
    


    # =============================== STOP ================================================
    def stop(self):
        brake(self.queuesList)
        
        self.queuesList[EngineRun.Queue.value].put(
        {
        "Owner": EngineRun.Owner.value,
        "msgID": EngineRun.msgID.value,
        "msgType": EngineRun.msgType.value,
        "msgValue": False
        }
        )
        super(threadMove_fast, self).stop()

    # =============================== CONFIG ==============================================
    #def Configs(self):
    #    """Callback function for receiving configs on the pipe."""

    # =============================== START ===============================================
    def start(self):
        super(threadMove_fast, self).start()
    
    # =============================== EMPTY RECEIVING PIPES ===============================
    
    def flush_all(self):
        for pipe in self.pipes:
            while pipe.poll():
                junk=pipe.recv()
                
    def Configs(self):
        """Function for receiving configs on the pipe."""
        while self.pipeRecvconfig.poll():
            message = self.pipeRecvconfig.recv()["value"]
            if (message["action"] == "autonomous"):
                self.autonomous = eval(message["value"])
            elif (message["action"] == "recording"):
                self.recording = eval(message["value"])
            elif (message["action"] == "speed"):
                self.speed = message["value"]
                print(self.speed)
            elif (message["action"] == "K_value"):
                self.K = float(message["value"])

    # ================================ RUN ================================================
    def run(self):
        "run function"

        
        
        time.sleep(0.5)
        # start_recording(self.queuesList)
        time.sleep(0.5)
        brake(self.queuesList)
        time.sleep(0.5)
        print("running")
        # logging.basicConfig(filename='example.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        # logging.info("-----------------------------NEW RUN---------------------------------------")
        # last_steer = 0
        
        "run function"

        directions = ['STRAIGHT', 'STRAIGHT', 'END']

        # ========== Flags Needed ========== #
        intersection_seen = False

    
        distance=0
        
        time.sleep(0.5) #wait for initializations of the other processes    

        while self._running:
            t0 = time.time()                
            """"
            If the engine button is pressed: 
            if engine button is on start running the car
            if the engine button is off stop the car
            """
            
            """
            TO DO
            Inside this if should start everything we need for the rest of the main flow (for example the path planning)
            """

            if (self.pipeRecvstart.poll()):
                self.engine = self.pipeRecvstart.recv()["value"]
                if self.engine:
                    # self.Configs()
                    self.flush_all()
                    for pipe in self.pipes:
                        pipe.send("ready")
                        
                      
                    setSpeed(self.queuesList, self.speed)
                    
                    if self.recording:
                        start_recording(self.queuesList)
                        time.sleep(0.5)
                else:
                    # disablelf(self.queuesList)
                    print("stopped running")
                    if self.recording:
                        stop_recording(self.queuesList)
                self.pipeRecvstart.send("ready")    
         
            
            if self.engine:
                
                "If the car runs autonomous"
                
                """
                TO DO
                in the exception check if we need to send ready to every pipe (or make any changes)
                """
                if self.autonomous:
                    
                    # ==================== Always do lanefollowing ==================== #
                    if self.pipeRecvcamera_lf.poll(): 
                        #print("LANE FOLLOWING")
                        frame = self.pipeRecvcamera_lf.recv()
                        image_data = base64.b64decode(frame["value"])
                        img = np.frombuffer(image_data, dtype=np.uint8)
                        image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                        data = lf.followLane(image, self.K, self.speed)
                        if(data is not None):
                            angle, lane_offset = data 
                            angle = np.clip(angle, -25, 25)
                            steer(self.queuesList, angle)

                        self.pipeRecvcamera_lf.send("ready")
                        # print("lane following time ", time.time() - t0)

                    # -------------------- Check: intersection detection -------------------- #
                    if (not intersection_seen):
                        # print("INTERSECTION REQUEST")
                        self.queuesList[Estimate.Queue.value].put( #send request to do intersection detection
                            {
                                "Owner": Estimate.Owner.value,
                                "msgID": Estimate.msgID.value,
                                "msgType": Estimate.msgType.value,
                                "msgValue": True
                            }
                        )

                    if self.pipeRecvInterDet.poll():
                        distance = self.pipeRecvInterDet.recv()["value"]
                        intersection_seen = True
                        print("intersection in distance = ", distance)
                        t = int(distance/(float(self.speed)/100))
                        start_time = time.time()

                    
                    if intersection_seen:   #made check earlier
                        if (time.time() - start_time <= t + 3.5  and ((self.pipeRecvinf.recv()["value"] == 0) or (time.time() - start_time <= t - 1))):
                            self.pipeRecvinf.send('ready')
                            continue
                        
                        brake(self.queuesList)
                        time.sleep(1)
                        current = directions.pop(0)
                        # if current == 'END':
                        #     brake(self.queuesList)
                        #     break
                        print(current)
                        setSpeed(self.queuesList, self.speed)
                        self.pipeIMUrecv.send("ready")
                        while(not self.pipeIMUrecv.poll()):
                            continue
                        if(self.pipeIMUrecv.poll()):
                            angle = float(self.pipeIMUrecv.recv()["value"]["yaw"])
                            angle = round90(angle) 
                        intersection_navigation(current, self.pipeIMUrecv, self.queuesList, extra_angle = angle, speed = self.speed)
                            
                    
                        
                        #reset flags 
                        intersection_seen = False
                        self.flush_all()
                        for pipe in self.pipes:
                            pipe.send("ready")
                
                
            