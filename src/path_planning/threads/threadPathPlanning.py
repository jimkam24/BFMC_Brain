import cv2
import threading
import base64
import networkx as nx
import time

import src.path_planning.threads.path_planning as pp 

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    Calculate,
    Path,
)
from src.templates.threadwithstop import ThreadWithStop


class node:
    def __init__(self, x, y, next):
        self.x = x
        self.y = y
        self.next = next
        self.dashed = False

class threadPathPlanning(ThreadWithStop):

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadPathPlanning, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.source = None       #source from which to calculate new path
        pipeRecvCalc, pipeSendCalc = Pipe()
        self.pipeRecvCalc = pipeRecvCalc
        self.pipeSendCalc = pipeSendCalc
        self.subscribe()
        self.pipeRecvCalc.send("ready")   #send ready flag through pipe

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Calculate.Owner.value,
                "msgID": Calculate.msgID.value,
                "To": {"receiver": "threadPathPlanning", "pipe": self.pipeSendCalc},
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
        super(threadPathPlanning, self).stop()

    # =============================== START ===============================================
    def start(self):
        super(threadPathPlanning, self).start()

    # =============================== CONFIG ==============================================
    # def Configs(self):
    #     """Callback function for receiving configs on the pipe."""

    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True"""
        while self._running:
            G = nx.read_graphml("Lab_track_graph.graphml")    #read graph
            target_nodes = [26,41,10,4,1]
            #target_nodes = [397,334]
            dashed = []
            dashed_sources = []

            for edge in G.edges(data=True):
                if "dotted" in edge[2] and edge[2]["dotted"] == True:
                    dashed.append(edge)         #edges over dashed lines (overtaking allowed)

            dashed_sources = [int(edge[0]) for edge in dashed]  #source nodes from which dashed lines to other nodes exist

            # Create a dictionary to store nodes
            graph = {}

            for n in G.nodes():
                node_coordinates = G.nodes[n]
                n = int(n)
                node_info = node(node_coordinates['x'], node_coordinates['y'], [int(node) for node in list(G.successors(str(n)))])
                if n in dashed_sources:
                    node_info.dashed = True     #from this node starts a dash line to any of its successors
                graph[n] = node_info
                
            if self.pipeRecvCalc.poll():
                self.source = int(self.pipeRecvCalc.recv()['value'])
                              
                path = []
                min_cost = float('inf')
                min_path = []
                full_path = []
                for i in range(len(target_nodes)): 
                    min_cost = float('inf')
                    for target in target_nodes:
                        path = list(map(int, nx.shortest_path(G, source=str(self.source), target=str(target), weight=None, method='dijkstra')))       #compute shortest path starting from node 472 (starting point)
                        cost = len(path)
                        if(cost<min_cost):
                            min_cost = cost
                            min_path = path
                    self.source = min_path[len(min_path)-1]
                    full_path += min_path[0:len(min_path)-1] #append paths to full path, without last node
                    target_nodes.remove(min_path[len(min_path)-1]) #remove chosen target
                full_path.append(min_path[len(min_path)-1]) #to append last node

                        #comment: list(map(int, ....)) converts the returned path nodes to integers, because they are returned as strings!!
                directions = []
                intersection = [49,8,78] #put all intersections
                roundabout = [357]
                print(full_path)
                directions = pp.find_directions(full_path, intersection,graph, roundabout) # find directions list



                message = (full_path, directions) #send tuple message : path and directions

                '''
                We can easily access information for the given path using the graph dictionary like so:

                - get the x coordinate of any given node n: graph[n].x
                - get the y coordinate of any given node n: graph[n].y
                - get the successors of any given node n: graph[element].next (list of nodes)
                - find out if the current node is part of a road with dashed lines or not: graph[element].dashed

                Uncomment the code below to get a ppretty printed preview of the path and its attributes
                '''
                # pp.print_path_info(G, graph, path)

                '''
                Uncomment the code below to get a preview of the map, with the calculated path shown with colour

                - Red nodes mean solid line
                - Dark blue nodes mean dashed line
                '''
                # pp.draw_path(G, graph, path)

                self.queuesList[Path.Queue.value].put(      #send back calculated path
                    {
                        "Owner": Path.Owner.value,
                        "msgID": Path.msgID.value,
                        "msgType": Path.msgType.value,
                        "msgValue": message
                    }
                )

                self.pipeRecvCalc.send("ready")   #send ready flag through pipe

                break