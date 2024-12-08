from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import sympy as smp
import math as ma
import time
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.intersection import round90

def draw_roundabout_trajectory(x, y, check, phi):
    C = 0.55
    radius = 65

    if(check == 1):
        points = np.array([
        [0, -1.9], [0.07, -1.5], [0.3,-1],[0.45, -0.9]]) * radius
    elif(check == 2):
        points = np.array([
        [0, -1.9], [0.1, -1.5], [0.4, -0.9], [0.6, -0.2], [0.55, 0.4], [0.25, 0.8], ]) * radius 
    elif(check == 3):
        points = np.array([
        [0, -1.9], [0.1, -1.5], [0.4, -1], [0.6, -0.2], [0, 1], [-0.4, 0.7], [-0.65, 0.17]]) * radius
    x, y = points.T
    # Creating a parametric spline. The 's' parameter controls the smoothness.
    # 'per' is set to True for a periodic spline (useful for closed paths)
    tck, u = splprep([x, y], s=1, per=False)

    # Generating new points along the spline for a smooth curve
    new_points = 100
    u_new = np.linspace(u.min(), u.max(), new_points)
    x_new, y_new = splev(u_new, tck)
    x_der, y_der = splev(u_new, tck, 1)

    # Assuming dx_new and dy_new are your derivative arrays as previously calculated
    angles = np.arctan2(y_der, x_der)
    # Convert from radians to degrees, if desired
    angles = np.degrees(angles)
    slopes = y_der
    distances = []
    i = 0
    for i in range(len(x_new)):
        if i + 1 < len(x_new):
            point1 = np.array([x_new[i], y_new[i]])
            point2 = np.array([x_new[i+1], y_new[i+1]])
            distances.append(np.linalg.norm(point1 - point2))
        else:
            break
        

    ret = -1*np.diff(angles)
    ret[0] = ret[0] - phi//2
    return ret, distances

def roundabout_reaction(queuesList, direction, pipe):
    
    angle_offset = 0
    pipe.send("ready")
    while(not pipe.poll()):
        continue
    if(pipe.poll()):
        angle_offset = float(pipe.recv()["value"]["yaw"])
        angle_offset = round90(angle_offset)
    
    if (direction == 'STRAIGHT'):
        angles, distances = draw_roundabout_trajectory(0, 0, 2, angle_offset)
        angle=0
        i = 0
        for dist in distances:
            t = dist / 15.0
            # if (angles[i] > 0 and angle < 0):
            #     angle = -7
            if (angles[i] < 0 and angle > 10):
                angle = 0
            else:
                angle = angle + angles[i]
            steer(queuesList, angle)
            print(angle)
            i = i + 1
            time.sleep(t)
        counter = 1
        steer(queuesList, 20)
        time.sleep(1.5)
    elif (direction == 'LEFT'):
        angles, distances = draw_roundabout_trajectory(0, 0, 3, angle_offset)
        angle=0
        i = 0
        for dist in distances:
            t = dist / 15.0
            if (angles[i] > 0 and angle < 0):
                angle = -7
            elif (angles[i] < 0 and angle > 15):
                angle = 0
            else:
                angle = angle + angles[i]
            steer(queuesList, angle)
            i = i + 1
            time.sleep(t)
        counter = 1
        steer(queuesList, 20)
        time.sleep(1.5)
    else:
        angles, distances = draw_roundabout_trajectory(0, 0, 1, angle_offset)
        angle=0
        i = 0
        for dist in distances:
            t = dist / 15.0
            angle = angle + angles[i]
            steer(queuesList, angle)
            i = i + 1
            time.sleep(t)
        counter = 1
        steer(queuesList, 20)
        time.sleep(1.5)
        