import serial
import time
import threading

import math
from sklearn.metrics import euclidean_distances
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

import matplotlib.image as mpimg


class Gps:
    def __init__(self):
        # Anchor node locations
        # anchor = #1 #2 #3 0,3860 3120,-1620 5960,3660
        self.anchor_x = [3120, 5960, 0]
        self.anchor_y = [-1590,3880,3690]
        self.count = 0
        self.samples_to_count = 100
        self.previous_value = 0
        self.total = np.array((0,0))
    

    def trilaterate(self, anchor1, anchor2, anchor3):
        """
        @brief: Trilaterate Tag location
        @param: anchor_x - List of anchor coordinates along the X-axis
                anchor_y - List of anchor coordinates along the Y-axis
                anchor1 - Distance to the 1st Anchor 
                anchor2 - Distance to the 2nd Anchor
                anchor3 - Distance to the 3rd Anchor
        @ret:   tag_coordinates - Tag Coordinates in a numpy array.
        """
        r1_sq = pow(anchor1,2)
        r2_sq = pow(anchor2,2)
        r3_sq = pow(anchor3,2)

        # Solve a linear matrix equation where x,y is the Tag coordinate:
        # Ax + By = C
        # Dx + Ey = F
        A = (-2*self.anchor_x[0]) + (2*self.anchor_x[1])
        B = (-2*self.anchor_y[0]) + (2*self.anchor_y[1])
        C = r1_sq - r2_sq - pow(self.anchor_x[0],2) + pow(self.anchor_x[1],2) - pow(self.anchor_y[0],2) + pow(self.anchor_y[1],2) 
        D = (-2*self.anchor_x[1]) + (2*self.anchor_x[2])
        E = (-2*self.anchor_y[1]) + (2*self.anchor_y[2])
        F = r2_sq - r3_sq - pow(self.anchor_x[1],2) + pow(self.anchor_x[2],2) - pow(self.anchor_y[1],2) + pow(self.anchor_y[2],2) 

        a = np.array([[A, B], [D, E]])
        b = np.array([C, F])
        tag_coordinates = np.linalg.solve(a, b)
        # print("Tag Coordinate:", tag_coordinates)
        return tag_coordinates

    # Filter out corrupted UART data and perform trilateration 
    def update(self,ser):
        value = str(ser.readline().decode('utf-8', errors='replace'))
        if(("0x92bb: =" in value) and (" | 0x9832: =" in value) and (" | 0x14a6: =" in value) and ("\r\n" in value)):
        # if(("4818" in value) and ("528d" in value) and ("84b9" in value) and ("\r\n" in value)):
            data = value.split(" | ")
            # print("Data: ", data)
            if(len(data) == 4): # check for 3 nodes and \r\n
                # print("data[0]: ", data[0])
                # print("data[1]: ", data[1])
                # print("data[2]: ", data[2])
                node1 = data[0].split("=")
                node2 = data[1].split("=")
                node3 = data[2].split("=")
                if (((len(node1) == 2) and (len(node2) == 2) and (len(node3) == 2)) and 
                ((node1[1].isnumeric() == 1) and (node2[1].isnumeric() == 1) and (node3[1].isnumeric() == 1))):
                    node1 = int(node1[1])
                    node2 = int(node2[1])
                    node3 = int(node3[1])
                    # print("node1: ", node1)
                    # print("node2: ", node2)
                    # print("node3: ", node3)
                    tag = self.trilaterate( node1, node2, node3)
                    # print("Tag Coordinate:", tag)
                    self.previous_value = tag
                    #global total
                    if self.count < self.samples_to_count:
                            self.total = self.total + tag
                            self.count += 1
                    if self.count == self.samples_to_count:
                        self.total /= self.samples_to_count
                        result = self.total
                        # print("Tag: ", total)
                        self.total = np.array((0,0))
                        self.count = 0
                        return result
                else:
                    #print("Node Length / Int Error")
                    tag = self.previous_value
            else:
                #print("3 node Error")
                tag = self.previous_value
        else:
            #print("Readline Error")
            tag = self.previous_value

