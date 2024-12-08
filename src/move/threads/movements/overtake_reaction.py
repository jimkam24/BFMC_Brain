from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from time import sleep
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.PID import PID
import numpy as np
from src.utils.messages.allMessages import (
    CurrentSpeed
)
import base64
import cv2 
import time

from src.move.threads.movements.lane_following import followLane

def overtake(queuesList, rdistpipe, imgpipe, K): #inPs,,queue,capture
    print("I am in overtake function")
    setSpeed(queuesList, 16)
    sleep(1)
    #lanefollowfor(inPs,queuesList,6,queue,capture)
    #brake(queuesList)
    # time.sleep(5)
    overtake_navigation(rdistpipe, imgpipe, queuesList, K)

    steer(queuesList,-20)
    setSpeed(queuesList,20)
    sleep(2)
    brake(queuesList)

  
def draw_left_overtake_trajectory(x, y, phi):
    l = 36.5
    w = 35.0
    ultrasonic_distance = 20.0 #ultrasonic info
    
    y = [0, -(l/2 + ultrasonic_distance) * (2/4), -(l/2 + ultrasonic_distance) * (3/4), -(l/2 + ultrasonic_distance)]
    x = [0, (w + 2.0)*(1/5), (w + 2.0)*(2/5), (w + 2.0)*(2/4)]
    yy = CubicSpline(x, y)
    xx = np.linspace(min(x), max(x), 5)


#    for i in range(len(x)):
#        plt.plot(x[i], y[i], 'ro')

#    plt.plot(xx, yy(xx), 'b-')
#    plt.grid()
#    plt.show()

    yy_der = yy.derivative()
    slopes = []
    distances = []
    i = 0
    for xes in xx:
        slopes.append(yy_der(xes))

    for i in range(len(xx)):
        if i + 1 < len(xx):
            point1 = np.array([xx[i], yy(xx[i])])
            point2 = np.array([xx[i+1], yy(xx[i+1])])
            distances.append(np.linalg.norm(point1 - point2))
        else:
            break

    angles_rad = np.arctan(slopes)
    angles = np.rad2deg(angles_rad)
    # for i in range(len(angles)):
    #     angles[i] = 90 - angles[i]

    return (-1) * np.diff(angles), distances


def draw_right_overtake_trajectory(x, y, phi):
    l = 36.5
    w = 35.0
    ultrasonic_distance = 20.0 #ultrasonic info
    
    y = [0, (l/2 + ultrasonic_distance) * (2/4) + 10, (l/2 + ultrasonic_distance) * (3/4) + 15, (l/2 + ultrasonic_distance) + 20]
    x = [0, (w + 2.0)*(1/5), (w + 2.0) *(2/5), (w + 2.0)*(2/4)]
   
    yy = CubicSpline(x, y)
    xx = np.linspace(min(x), max(x), 5)

 #   for i in range(len(x)):
  #      plt.plot(x[i], y[i], 'ro')
  #  plt.plot(xx, yy(xx), 'b-')
  #  plt.grid()
   # plt.show()

    yy_der = yy.derivative()
    slopes = []
    distances = []
    i = 0
    for xes in xx:
        slopes.append(yy_der(xes))

    for i in range(len(xx)):
        if i + 1 < len(xx):
            point1 = np.array([xx[i], yy(xx[i])])
            point2 = np.array([xx[i+1], yy(xx[i+1])])
            distances.append(np.linalg.norm(point1 - point2))
        else:
            break

    angles_rad = np.arctan(slopes)
    angles = np.rad2deg(angles_rad)
    
    return (-1)*np.diff(angles), distances

def overtake_navigation(rdistpipe, imgpipe, queuesList, K, speed = 15):
    print("I am in overtake navigation")
    count = 0
    #d_forward = take distance from forward ultrasonic 
    #d_right = take distance from right ultrasonic

    angles, distances =  draw_left_overtake_trajectory(0, 0, 0)
    i= 0
    angle=0
    for dist in distances:
        t = dist / 15.0
        angle =angle + angles[i]
        steer(queuesList, angle)
        i = i +1
        sleep(t)
    
    steer(queuesList, 17)
    time.sleep(3)
    
    r_distance = 200
    if rdistpipe.poll():
        r_distance = rdistpipe.recv()["value"]
        print(f"Right distance is {r_distance}")
        rdistpipe.send("ready")
  

    while(count < 4):
        print(f"I am in overtake and my count is {count}")
        if imgpipe.poll():
            frame = imgpipe.recv()
            image_data = base64.b64decode(frame["value"])
            img = np.frombuffer(image_data, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            data = followLane(img,K, speed)
            if data is not None:
                angle, _ = data
            steer(queuesList, angle)
            imgpipe.send("ready")
        if rdistpipe.poll():
            r_distance = rdistpipe.recv()["value"]
            print(f"Right distance is {r_distance}")
            rdistpipe.send("ready")
            if (r_distance > 35.0):
                count = count + 1
    
    count = 0
    print("I exited while loop")
    '''
    while(count < 12):
        if imgpipe.poll():
            frame = imgpipe.recv()
            image_data = base64.b64decode(frame["value"])
            img = np.frombuffer(image_data, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            data = followLane(img,K, speed)
            if data is not None:
                angle, _ = data
            steer(queuesList, angle)
            imgpipe.send("ready")
        if rdistpipe.poll():
            r_distance = rdistpipe.recv()["value"]
            print(f"Right distance is {r_distance}")
            rdistpipe.send("ready")
        if (r_distance < 35.0):
            count = count + 1
    '''
    print("hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
    angles, distances = draw_right_overtake_trajectory(0, 0, 0)
    i = 0
    angle=0
    for dist in distances:
        t = dist / 10.0
        print(t)
        angle = angle + angles[i]
        steer(queuesList, angle+15)
        i = i + 1
        sleep(t)

    time.sleep(0.5)
    steer(queuesList, -12)
    time.sleep(2)
    print("finished")
    '''
    while(True):
        if imgpipe.poll():
            frame = imgpipe.recv()
            image_data = base64.b64decode(frame["value"])
            img = np.frombuffer(image_data, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            data = followLane(img,K, speed)
            if data is not None:
                angle, _ = data
            steer(queuesList, angle)
            imgpipe.send("ready")
    '''
    print('Exited overtake!!')
