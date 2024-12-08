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
import networkx as nx

from multiprocessing import Pipe
from src.utils.messages.allMessages import ( 
    serialCamera,
    FollowLane
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
import src.move.threads.movements.lane_following as lf

class threadLaneFollowing(ThreadWithStop):
    """Thread which will handle the lane following.\n
    Args:
        pipeRecv: maybe
        pipeSend: maybe
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadLaneFollowing, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.enable = False
        
        # Setting the parameters for lane following
        self.K = 0.11 #0.12 or 0.15 maybe works
        self.speed = 16 #15 is default
        

        
        self.pipes = list()
        pipeRcvenablelf, pipeSendenablelf = Pipe()
        pipeRecvcamera, pipeSendcamera = Pipe()       
        self.pipeRecvcamera = pipeRecvcamera
        self.pipeSendcamera = pipeSendcamera
        self.pipeRecvenablelf = pipeRcvenablelf
        self.pipeSendenablelf = pipeSendenablelf
        # self.pipeRecvenablelf.send("ready")
        # self.pipeRecvcamera.send("ready")
        self.subscribe()

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": serialCamera.Owner.value,
                "msgID": serialCamera.msgID.value,
                "To": {"receiver": "threadLaneFollowing", "pipe": self.pipeSendcamera},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": FollowLane.Owner.value,
                "msgID": FollowLane.msgID.value,
                "To": {"receiver": "threadLaneFollowing", "pipe": self.pipeSendenablelf},
            }
        )
        

    # =============================== STOP ================================================
    def stop(self):
        super(threadLaneFollowing, self).stop()

    # =============================== START ===============================================
    def start(self):
        super(threadLaneFollowing, self).start()
                
    
    # ================================ RUN ================================================
    def run(self):
        print("running")
        self.pipeRecvenablelf.send("ready")
        self.pipeRecvcamera.send("ready")
        while self._running:
            if self.pipeRecvenablelf.poll():
                self.enable = self.pipeRecvenablelf.recv()["value"]
                self.pipeRecvenablelf.send("ready")
            
            if self.pipeRecvcamera.poll() and self.enable: 
                t0 = time.time()
                frame = self.pipeRecvcamera.recv()
                image_data = base64.b64decode(frame["value"])
                img = np.frombuffer(image_data, dtype=np.uint8)
                image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                data = lf.followLane(image, self.K, self.speed)
                if(data is not None):
                    angle, lane_offset = data 
                    angle = np.clip(angle, -25, 25)
                    steer(self.queuesList, angle)

                self.pipeRecvcamera.send("ready")
                print("lane following time:  ", time.time() - t0)
