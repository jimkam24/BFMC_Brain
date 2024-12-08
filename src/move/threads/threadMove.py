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
    ImuData, 
    serialCamera,
    EngineRun,
    TrafficSign,
    Pedestrian,
    Path,
    Estimate,
    InterDistance,
    MoveConfig,
    Pos,
    FrontDistance,
    Semaphores,
    CurrentSpeed,
    Calculate,
    RightDistance,
    SignsSearching,
    LeftDistance,
    Cars,
    Infrared,
    FollowLane,
    Location
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording, enablelf, disablelf
import src.move.threads.movements.lane_following as lf
from src.move.threads.movements.sign_reaction import sign_reaction
from src.move.threads.movements.intersection import gostraight, draw_trajectory, intersection_navigation, round90
from src.move.threads.movements.roundabout import roundabout_reaction
from src.move.threads.movements.tailing import tail
from src.move.threads.movements.overtake_reaction import overtake
from src.data.CarsAndSemaphores.threads.Semaphores import Semaphores_reaction
from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.data.TrafficCommunication.threads.threadTrafficCommunicaiton import (
    threadTrafficCommunication,
)



class threadMove(ThreadWithStop):
    """Thread which will handle the decision making.\n
    Args:
        pipeRecv: maybe
        pipeSend: maybe
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadMove, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        
        # setting the parameters needed
        self.engine = False
        
        # Setting the parameters for lane following
        self.K = 0.11 #0.12 or 0.15 maybe works
        self.speed = 16 #15 is default
        
        # flags
        self.autonomous = True
        self.recording = False

        self.current_node = None #initialize current node
        self.coordinates = None
        self.path = []
        self.directions = []

        #critical points
        self.stop_pos = [*range(44,50,1),*range(8,15,1), *range(74,79,1)]
        self.prioriy_pos = [*range(44,50,1),*range(8,15,1), *range(74,79,1)]
        self.parking_pos =[*range(22,38,1), *range(66,76,1)]
        self.crosswalk_pos = [*range(14,20,1), *range(51,60,1)]
        self.intersection = [49,8,78]
        self.highway_exit_pos = [] #[*range(8,15,1)] #[]
        self.highway_entry_pos = [] #[*range(14,20,1), *range(22,29,1)]#[]
        self.roundabout_pos = [*range(228,239,1), *range(249,259,1), *range(325,332,1), 371, *range(393,399,1) ,357, *range(349,357,1) ,370]
        
        self.pipes = list()
        pipeRecvstart, pipeSendstart = Pipe()
        self.pipeRecvstart = pipeRecvstart
        self.pipeSendstart = pipeSendstart
        self.pipeRecvstart.send("ready")
        # self.pipeRecvstart will not be appended to self.pipes, we want it to work separately
        pipeRecvconfig, pipeSendconfig = Pipe()
        self.pipeRecvconfig = pipeRecvconfig
        self.pipeSendconfig = pipeSendconfig
        self.pipeRecvconfig.send("ready")
        # same for pipeRecvconfig
        
        pipeRecvPathPlanning, pipeSendPathPlanning = Pipe(duplex = True)
        pipeRecvInterDet, pipeSendInterDet = Pipe(duplex = True)
        pipeRecvcamera_lf, pipeSendcamera_lf = Pipe()
        pipeIMUrecv, pipeIMUsend = Pipe()
        pipeRecvPos, pipeSendPos = Pipe()
        pipeRecvinf, pipeSendinf = Pipe()
        pipeRecvsemaphores, pipeSendsemaphores = Pipe()
        self.pipeRecvsemaphores = pipeRecvsemaphores
        self.pipeSendsemaphores = pipeSendsemaphores
        pipeRecvCars, pipeSendCars = Pipe()
        self.pipeRecvCars = pipeRecvCars
        self.pipeSendCars = pipeSendCars
        self.pipes.append(self.pipeRecvCars)
        pipeRecvfdist, pipeSendfdist = Pipe()
        pipeRecvrdist, pipeSendrdist = Pipe()
        pipeRecvldist, pipeSendldist = Pipe()
        pipeRecvSignSearching, pipeSendSignSearching = Pipe(duplex = True)
        self.pipeRecvrdist = pipeRecvrdist
        self.pipeSendrdist = pipeSendrdist
        self.pipes.append(self.pipeRecvrdist)
        self.pipeRecvldist = pipeRecvldist
        self.pipeSendldist = pipeSendldist
        self.pipes.append(self.pipeRecvldist)
        self.pipes.append(self.pipeRecvsemaphores)
        self.pipeRecvfdist = pipeRecvfdist
        self.pipeSendfdist = pipeSendfdist
        self.pipes.append(self.pipeRecvfdist)
        self.pipeIMUrecv = pipeIMUrecv
        # self.pipes.append(self.pipeIMUrecv)
        self.pipeIMUsend = pipeIMUsend       
        self.pipeRecvcamera_lf = pipeRecvcamera_lf
        self.pipes.append(self.pipeRecvcamera_lf)
        self.pipeSendcamera_lf = pipeSendcamera_lf
        piperecvTrSigns, pipesendTrSigns = Pipe()
        self.piperecvTrSigns = piperecvTrSigns
        self.pipes.append(self.piperecvTrSigns)
        self.pipesendTrSigns = pipesendTrSigns
        piperecvPed,pipesendPed = Pipe()
        self.piperecvPed = piperecvPed
        self.pipesendPed = pipesendPed
        self.pipes.append(self.piperecvPed)
        self.pipeRecvPathPlanning = pipeRecvPathPlanning
        self.pipeSendPathPlanning = pipeSendPathPlanning
        self.pipes.append(self.pipeRecvPathPlanning)
        self.pipeRecvInterDet = pipeRecvInterDet
        self.pipeSendInterDet = pipeSendInterDet
        self.pipes.append(self.pipeRecvInterDet)
        self.pipeRecvPos = pipeRecvPos
        self.pipeSendPos = pipeSendPos
        self.pipes.append(self.pipeRecvPos)
        self.pipeRecvSignSearching = pipeRecvSignSearching
        self.pipeSendSignSearching = pipeSendSignSearching
        self.pipes.append(self.pipeRecvSignSearching)
        self.pipeRecvinf = pipeRecvinf
        self.pipeSendinf = pipeSendinf
        locsysReceivePipe, locsysSendPipe = Pipe()
        self.locsysReceivePipe = locsysReceivePipe
        self.locsysSendPipe = locsysSendPipe
        self.pipes.append(self.locsysReceivePipe)
        self.pipes.append(self.pipeRecvinf)
        self.subscribe()
        # for pipe in self.pipes:
        #     pipe.send("ready")

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": MoveConfig.Owner.value,
                "msgID": MoveConfig.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendconfig},
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
                "Owner": serialCamera.Owner.value,
                "msgID": serialCamera.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendcamera_lf},
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
                "Owner": TrafficSign.Owner.value,
                "msgID": TrafficSign.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipesendTrSigns},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Pedestrian.Owner.value,
                "msgID": Pedestrian.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipesendPed},
            }
        )    
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Path.Owner.value,
                "msgID": Path.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendPathPlanning},
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
                "Owner": Pos.Owner.value,
                "msgID": Pos.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendPos},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": FrontDistance.Owner.value,
                "msgID": FrontDistance.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendfdist},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Semaphores.Owner.value,
                "msgID": Semaphores.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendsemaphores},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Cars.Owner.value,
                "msgID": Cars.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendCars},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": RightDistance.Owner.value,
                "msgID": RightDistance.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendrdist},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": LeftDistance.Owner.value,
                "msgID": LeftDistance.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendldist},
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

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Location.Owner.value,
                "msgID": Location.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.locsysSendPipe},
            }
        )


    # =============================== STOP ================================================
    def stop(self):
        super(threadMove, self).stop()

    # =============================== CONFIG ==============================================
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

    # =============================== START ===============================================
    def start(self):
        super(threadMove, self).start()
    
    # =============================== EMPTY RECEIVING PIPES ===============================
    
    def flush_all(self):
        for pipe in self.pipes:
            if pipe.poll():
                junk=pipe.recv()
                
    
    # ================================ RUN ================================================
    def run(self):
        "run function"
        
        " Here go the necessary initializations before starting the loop of the main flow"
        semaphores_position = {} #storing position and id of traffic lights
        count = 4
        self.pipeRecvsemaphores.send("ready")           
        id_list = [0,1,2,3]

        closest_node_tr = None
        min_distance_tr = float('inf')
        closest_node = None

        G = nx.read_graphml("Lab_track_graph.graphml")

        
        #initialize current node
        same_node_counter = 0
        initialize_pos = False
        initialization = True #for first loop only

        directions = []

        # ========== Flags Needed ========== #
        intersection_seen = False
        sign_seen = False
        reduced_speed = False
        parking_found = False
        parking_search = True
        tailing = False
        counter = 0
        
        lane_offset = 0
        
        A = 0.25
        distance_left=0
        EAM_left=50
        distance_right = 0
        EAM_right = 50
        #start of parking spot
        parking_time=0
        count_parking_left = 0
        count_parking_right = 0
        counttail = 0
        right = False
        left = False
        
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
                    self.Configs()
                    self.flush_all()
                    for pipe in self.pipes:
                        pipe.send("ready")
                        
                    if self.autonomous:
                

                        self.queuesList[CurrentSpeed.Queue.value].put( #send current velocity to do position calculation
                                {
                                    "Owner": CurrentSpeed.Owner.value,
                                    "msgID": CurrentSpeed.msgID.value,
                                    "msgType": CurrentSpeed.msgType.value,
                                    "msgValue": 0.0
                                }   
                            )

                        while initialize_pos == False:
                        
                            if self.pipeRecvPos.poll():
                                self.coordinates = self.pipeRecvPos.recv()['value']
                                #find nearest node to current coordinates
                                min_distance = float('inf')
                                for node in G.nodes():
                                    node_coordinates = (G.nodes[node]['x'], G.nodes[node]['y'])  # Assuming nodes have 'x' and 'y' attributes
                                    distance = np.sqrt((self.coordinates[0] - node_coordinates[0])**2 + (self.coordinates[1] - node_coordinates[1])**2)
                                    if distance < min_distance:
                                        min_distance = distance
                                        closest_node = int(node)

                                print(closest_node, self.coordinates)

                                self.pipeRecvPos.send("ready")
                                
                                if closest_node == self.current_node:
                                    same_node_counter += 1
                                else:
                                    same_node_counter = 0  # Reset the counter if the node changes
                                    self.current_node = closest_node  # Update the current node
                                
                                if same_node_counter == 8:
                                    initialize_pos = True

                        # self.current_node = 387
                        if (initialize_pos == True and initialization == True):   

                            self.queuesList[Calculate.Queue.value].put( #send request to do path calculation
                                {
                                    "Owner": Calculate.Owner.value,
                                    "msgID": Calculate.msgID.value,
                                    "msgType": Calculate.msgType.value,
                                    "msgValue": self.current_node       #source node
                                }   
                            ) 

                            print("Starting node: ", self.current_node)
                            while not self.pipeRecvPathPlanning.poll():    #recieve path to follow
                                continue

                            (self.path, self.directions) = self.pipeRecvPathPlanning.recv()['value']
                            print("Path to follow: ", self.path)
                            self.pipeRecvPathPlanning.send("ready")

                            directions = self.directions
                            print("Directions : ", directions)
                            
                            initialization = False      #position initialization is over
                        
                        #==================================== WE HAVE A PATH TO FOLLOW =========================================
                        
                        
                        # -------------------- Wait for traffic lights -------------------- #
                            
                        count = 4 
                        while (False):
                            if self.pipeRecvsemaphores.poll(): 
                                Semaphore = (self.pipeRecvsemaphores.recv())
                                #Store the position of all sempaphores
                                if(Semaphore["value"]["id"] in id_list):
                                    print("here")
                                    id_list.remove(Semaphore["value"]["id"])
                                    count = count -1
                                    for node in G.nodes():
                                        node_coordinates = (G.nodes[node]['x'], G.nodes[node]['y'])  # Assuming nodes have 'x' and 'y' attributes
                                        distance = np.sqrt((float(Semaphore["value"]["x"]) - node_coordinates[0])**2 + (float(Semaphore["value"]["y"])- node_coordinates[1])**2)
                                        if distance < min_distance_tr:
                                            min_distance_tr = distance
                                            closest_node_tr = node
                                    semaphores_position.update({closest_node_tr:Semaphore["value"]["id"]})
                                    #FIND PREVIOUS NODES ---> FOR TRAFFIC LIGHTS
                                    count_prev = 0
                                    while(True):
                                        for source, target in G.edges():
                                            if target == closest_node_tr:
                                                semaphores_position.update({source:Semaphore["value"]["id"]})
                                                count_prev += 1
                                                closest_node_tr = source
                                                break
                                        if count_prev == 4:  # Stop after finding four previous nodes
                                            break  

                                #Check if the semaphore with id = 0 is green, then start
                                if(Semaphore["value"]["id"] ==  0 and count ==0):
                                    print("0")
                                    if (Semaphore["value"]["state"]=="green"):
                                        print("LECLERC GOES")
                                        break
                                self.pipeRecvsemaphores.send("ready")

                    print("Sempahores ",semaphores_position)
                    print("running")
                    
                    # enablelf(self.queuesList)                         
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

                    # ==================== checks ==================== #
                        
                    #------------------- Check number 0: localization ----------------------#
                    if self.pipeRecvPos.poll():
                        #print("LOCALIZATION")
                        coordinates = self.pipeRecvPos.recv()['value']

                        self.coordinates = (coordinates[0], coordinates[1])

                        connected_nodes = list(G.neighbors(str(self.current_node)))
                        connected_nodes.append(str(self.current_node))
                        # print(connected_nodes)


                        #find nearest node to current coordinates
                        min_distance = float('inf')

                        for node in connected_nodes:
                            node_coordinates = (G.nodes[node]['x'], G.nodes[node]['y'])  # Assuming nodes have 'x' and 'y' attributes
                            distance = np.sqrt((coordinates[0] - node_coordinates[0])**2 + (coordinates[1] - node_coordinates[1])**2)
                            if distance < min_distance:
                                min_distance = distance
                                closest_node = node

                        self.current_node = int(closest_node)

                        print("Current node:", self.current_node, "at coordinates:", coordinates, " | Car is on the path" if int(self.current_node) in self.path else " | Car is lost")

                        self.pipeRecvPos.send("ready")

                    
                    # -------------------- Check number 1: intersection detection -------------------- #
                    if (not intersection_seen) and ((self.current_node in self.stop_pos) or (self.current_node in self.prioriy_pos) or (self.current_node in self.parking_pos) or (self.current_node in self.crosswalk_pos)): #check if in correct nodes for intersection searching
                        # print("INTERSECTION REQUEST")
                        self.queuesList[Estimate.Queue.value].put( #send request to do intersection detection
                            {
                                "Owner": Estimate.Owner.value,
                                "msgID": Estimate.msgID.value,
                                "msgType": Estimate.msgType.value,
                                "msgValue": True
                            }
                        )

                    if self.pipeRecvInterDet.poll() and ((self.current_node in self.stop_pos) or (self.current_node in self.prioriy_pos) or (self.current_node in self.crosswalk_pos)): #check if in correct nodes for intersection searching
                        distance = self.pipeRecvInterDet.recv()["value"]
                        intersection_seen = True
                        print("intersection in distance = ", distance)
                        t = int(distance/(float(self.speed)/100))
                        start_time = time.time()
                        
                    # -------------------- Check number 2: traffic signs -------------------- #
                    
                    if self.piperecvTrSigns.poll():            #this will be true only if detection was requested before
                        sign=self.piperecvTrSigns.recv()["value"]
                        sign_seen = True
                        if(self.current_node in self.parking_pos  and (sign != "Parking" )):
                            sign_seen = False
                            
                        
                    # -------------------- Check number 3: pedestrian -------------------- #
                    if self.piperecvPed.poll(): #check for pedestrian anywhere
                        Pedestrian =self.piperecvPed.recv()["value"]
                        self.piperecvPed.send("ready")
                        if(not intersection_seen and Pedestrian == True): #to avoid stopping before crosswalk
                            print("seen pedestrian")
                            # disablelf(self.queuesList)
                            brake(self.queuesList)
                            time.sleep(2)

                            if not self.piperecvPed.poll():
                                # enablelf(self.queuesList)
                                setSpeed(self.queuesList, self.speed)
                    # ------------------- TAILING AND OVERTAKE --------------------------#
                    if self.pipeRecvfdist.poll():
                        front_dist = self.pipeRecvfdist.recv()["value"]
                        self.pipeRecvfdist.send("ready")
                        # print("dist",front_dist)

                        if front_dist < 80:
                            counttail = counttail + 1
                       # else:
                       #     counttail = 0
                        if counttail >=4:
                            if self.piperecvPed.poll():     # ????????????????????????????
                                pedestrian_seen = self.piperecvPed.recv()["value"]
                                self.piperecvPed.send("ready")
                                print("seen pedestrian")
                                brake(self.queuesList)
                                time.sleep(2)

                        #    else:
                                # setSpeed(self.queuesList, self.speed)
                        #        continue

                            # no pedestrian, so car
                            steer(self.queuesList, 0)

                            overtake_now = tail(self.pipeRecvfdist, self.queuesList, self.pipeRecvcamera_lf, self.pipeRecvCars, self.coordinates[0], self.coordinates[1], self.K, 100)
                            setSpeed(self.queuesList, self.speed)
                            counttail = 0
                            if overtake_now is True:
                                overtake(self.queuesList, self.pipeRecvfdist, self.pipeRecvcamera_lf, self.K)
                                setSpeed(self.queuesList, self.speed)
                    '''
                    # ------------------- Check number 5: tailing conditions ----------------------#
                    if self.pipeRecvfdist.poll(): #check for tailing not available yet...
                        front_dist = self.pipeRecvfdist.recv()["value"]
                        self.pipeRecvfdist.send("ready")
                        if(front_dist < 80):
                            counttail = counttail + 1
                        else:
                            counttail = 0
                        if(counttail>=4):
                            steer(self.queuesList, 0)
                            steer(self.queuesList, 0)
                            tail(self.pipeRecvfdist, self.queuesList, self.pipeRecvcamera_lf, self.K, 100)
                    '''    
                    #-------------------- Check number 6: Cars -------------------- #
                    #if self.pipeRecvCars.poll():
                        #car_pos = self.pipeRecvCars.recv()
                        # print("car :", car_pos)           
                
                    # ==================== Use the flags to find if we need a reaction ==================== #
                    # =================== Highway Reaction==================================================#
                    if (sign_seen == False):
                        if((self.current_node in self.highway_entry_pos) or (self.current_node in self.highway_exit_pos)):
                            continue

                    elif (sign == "highway_entry") and (self.current_node in self.highway_entry_pos):
                        print("seen sign:", sign)
                        self.speed = 25
                        sign_reaction(self.queuesList, "highway_entry", pipe = None)
                        sign_seen = False
                        self.piperecvTrSigns.send("ready")
                        continue

                    elif (sign == "highway_exit") and (self.current_node in self.highway_exit_pos):
                        print("seen sign:", sign)
                        self.speed = 16
                        sign_reaction(self.queuesList, "highway_exit", pipe = None)
                        sign_seen = False
                        self.piperecvTrSigns.send("ready")
                        continue
                    
                    elif (sign == "Parking") and (self.current_node in self.parking_pos):
                            # print("seen sign:", sign)
                            # find_parking()
                            if(int(self.current_node) != 33  and parking_search):
                                print("searching node")
                                continue
                            parking_search = False
                            if (not parking_found):
                                if (self.pipeRecvrdist.poll()):
                                    distance_right = float(self.pipeRecvrdist.recv()["value"])
                                    if distance_right > 100:
                                        distance_right = 100
                                    self.pipeRecvrdist.send("ready")
                                    EAM_right=(1-A)*EAM_right+A*distance_right
                                    print("EAM_right", EAM_right)

                                if (self.pipeRecvldist.poll()):
                                    distance_left = float(self.pipeRecvldist.recv()["value"])
                                    if distance_left > 100:
                                        distance_left = 100
                                    self.pipeRecvldist.send("ready")
                                    EAM_left=(1-A)*EAM_left+A*distance_left
                                    print("EAM_left", EAM_left)
                                
                                    if(EAM_right>60):
                                        if(count_parking_right==0):
                                            parking_time=time.time()
                                        count_parking_right+=1
                                        print("count right=", count_parking_right)
                                        # print("right distance = ", distance_right)
                                    else:
                                        count_parking_right=0

                                    if(EAM_left>60):
                                        if(count_parking_left==0):
                                            parking_time=time.time()
                                        count_parking_left+=1
                                        print("count left=", count_parking_left)
                                        # print("left distance = ", distance_left)
                                    else:
                                        count_parking_left=0
                                

                                if(count_parking_right>=10 and not left):
                                    print("right")
                                    dt = time.time() - parking_time
                                    if(dt>-1):
                                        parking_found = True
                                        right = True
                                        left = False
                                        parking_time2 = time.time()
                                        #start parking
                                    # else:
                                        # parking_time=time.time()

                                if(count_parking_left>=10 and not right):
                                    print("left")
                                    dt = time.time() - parking_time
                                    if(dt>-1):
                                        parking_found = True
                                        right = False
                                        left = True
                                        parking_time2 = time.time()
                                        #start parking
                                    # else:
                                        # parking_time=time.time()
                                
                                continue
                            else:
                                t0 = (115/15) - dt
                                if (time.time() - parking_time2 < t0):
                                    continue
                                brake(self.queuesList)
                                time.sleep(1)
                                steer(self.queuesList, 0)
                                time.sleep(1)
                                print("lane offset", lane_offset)
                                if (right):
                                    sign_reaction(self.queuesList, "Parking", pipe = [self.pipeIMUrecv, self.pipeRecvinf], offset = lane_offset, pos = 'RIGHT')
                                # enablelf(self.queuesList)
                                else:
                                    sign_reaction(self.queuesList, "Parking", pipe = [self.pipeIMUrecv, self.pipeRecvinf], offset = lane_offset, pos = 'LEFT')
                                setSpeed(self.queuesList, self.speed)
                                sign_seen = False
                                self.piperecvTrSigns.send("ready")
                                parking_search = True
                                parking_found = False
                                self.flush_all()
                                for pipe in self.pipes:
                                    pipe.send("ready")
                            continue

                    
                    if intersection_seen:   #made check earlier
                        if (time.time() - start_time <= t + 3.5 and ((self.pipeRecvinf.recv()["value"] == 0) or (time.time() - start_time <= t - 1))):
                            if((sign_seen and sign == "Crosswalk") and (not reduced_speed) and (self.current_node in self.crosswalk_pos)):
                                self.piperecvPed.send("ready")
                                print("seen sign:", sign)
                                self.speed = 10
                                setSpeed(self.queuesList, 10)
                                t = t + t/2
                                reduced_speed = True
                            self.pipeRecvinf.send('ready')
                            continue
                        
                        if (str(self.current_node) in semaphores_position):
                            print("SEMAPHORE")
                            self.pipeRecvsemaphores.send("ready")   
                            while(not self.pipeRecvsemaphores.poll()):
                                continue
                            while (True): #include position checking fro traffic light as well
                                if(self.pipeRecvsemaphores.poll()):
                                    Semaphore = (self.pipeRecvsemaphores.recv())
                                    if Semaphore["value"]["id"] == semaphores_position[str(self.current_node)]:
                                        if(Semaphore["value"]["state"]=="green"):
                                            print("GREEN LIGHT")
                                            intersection_seen = False
                                            break
                                        if((Semaphore["value"]["state"]=="red") or (Semaphore["value"]["state"]=="yellow")):
                                            print("RED LIGHT")
                                            brake(self.queuesList)
                                            time.sleep(0.5)
                                            steer(self.queuesList,0)
                                    self.pipeRecvsemaphores.send("ready")           
                            
                            setSpeed(self.queuesList,self.speed)
                            self.pipeIMUrecv.send("ready")
                            while(not self.pipeIMUrecv.poll()):
                                continue
                            if(self.pipeIMUrecv.poll()):
                                angle = float(self.pipeIMUrecv.recv()["value"]["yaw"])
                                angle = round90(angle)
                            current = directions.pop(0)
                            intersection_navigation(current, self.pipeIMUrecv, self.queuesList, extra_angle = angle)
                            continue
                
                        if (sign_seen == False ):
                            if(self.current_node not in self.parking_pos):
                                print("no signnn")
                                current = directions.pop(0)
                                setSpeed(self.queuesList, self.speed)
                                self.pipeIMUrecv.send("ready")
                                while(not self.pipeIMUrecv.poll()):
                                    continue
                                if(self.pipeIMUrecv.poll()):
                                    angle = float(self.pipeIMUrecv.recv()["value"]["yaw"])
                                    angle = round90(angle) 
                                intersection_navigation(current, self.pipeIMUrecv, self.queuesList, extra_angle = angle)
                            
                        elif (sign == "Priority" or sign == "Stop") and (self.current_node in self.stop_pos or self.prioriy_pos):
                            print("Seen sign:", sign)
                            sign_reaction(self.queuesList, sign)
                            setSpeed(self.queuesList, self.speed)
                            current = directions.pop(0)
                            self.pipeIMUrecv.send("ready")
                            while(not self.pipeIMUrecv.poll()):
                                continue
                            if(self.pipeIMUrecv.poll()):
                                angle = float(self.pipeIMUrecv.recv()["value"]["yaw"])
                                angle = round90(angle) 
                            intersection_navigation(current, self.pipeIMUrecv, self.queuesList, extra_angle = angle)
                        
                        elif (sign == "Crosswalk") and self.current_node in self.crosswalk_pos:
                            print("CROSSWALK")
                            brake(self.queuesList)
                            time.sleep(1)
                            steer(self.queuesList, 0)
                            sign_reaction(self.queuesList, sign, self.piperecvPed)
                            self.speed = 16
                            setSpeed(self.queuesList, self.speed)
                            gostraight(self.pipeIMUrecv, self.queuesList, int(70/int(self.speed)))
                            
                        # else:  #for rest of signs (e.g highway), to be completed later...
                        #     print("seen sign:", sign)
                        #     sign_reaction(self.queuesList, sign)
                    
                        
                        #reset flags 
                        parking_found = False
                        sign_seen = False
                        intersection_seen = False
                        reduced_speed = False
                        self.flush_all()
                        for pipe in self.pipes:
                            pipe.send("ready")
        
                # print("while loop timeeeeeee :  " , time.time() - t0)
                # -------->> EVERYTHING AFTER "else" IS FOR TESTING
                else:                  
                    # uncomment for roundabout reaction (choose direction)
                    roundabout_reaction(self.queuesList, 'STRAIGHT', pipe = self.pipeIMUrecv, speed = 20)
                    brake(self.queuesList)
                    break                    
                     
                    
                    # uncomment for servers
                    # m = "historyData"
                    # from multiprocessing import Queue, Event
                    # #import time
                    # from src.utils.messages.allMessages import  Location

                    # self.locsysReceivePipe.send("ready")
                    # x,y = self.locsysReceivePipe.recv()['value']
                    # self.locsysReceivePipe.send("ready")
                    # print(x,y)

                    
                   