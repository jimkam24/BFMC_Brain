import time
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.PID import PID
from scipy.interpolate import CubicSpline
import numpy as np

from src.utils.messages.allMessages import (
    CurrentSpeed
)

def round90(angle):
    d = angle//90
    if d == 0:
        return angle
    elif angle%90>45:
        return angle - d*90 - 90
    else:
        return  angle - (d-1)*90 - 90

def find_targ(angle, direction):
    print("angle ", angle)
    offset = angle%90
    if offset > 45:
        offset = offset - 90
    if (direction == "RIGHT"):
        target = angle + 90 - offset
    elif (direction == "LEFT"):
        target = angle - 90 - offset
    elif (direction == "STRAIGHT"):
        target = angle - offset
    if target < 0:
        target = target + 360
    if target >=360:
        target = target - 360
    print("target= ", target)
    return target

def gostraight(pipe, queuesList, t):
    if (pipe.poll()):
        dummy = pipe.recv()
    pipe.send("ready")
    while (not pipe.poll()):
        continue
    target = find_targ(float(pipe.recv()["value"]["yaw"]), "STRAIGHT")
    pipe.send("ready")
    start_time = time.time()
    pid = PID((4, 0.0, 0.05), target, [-15, 15])
    while (time.time() - start_time <= t):
        if pipe.poll():
            data = pipe.recv()["value"]
            if (float(data["yaw"]) > 315):
                angle = float(data["yaw"]) - 360
            else:
                angle = float(data["yaw"])
            print("angle", angle)
            pipe.send("ready")
            angle = pid.update(angle)
            print("steer", angle)
            steer(queuesList, angle)

def draw_trajectory(xs, xf, phi, offset = 0, extra_angle = 0):
    x = [0, 20 - offset, 68 - offset]
    y = [0, 18 - offset, 25 - offset]
    xx = np.linspace(min(x), max(x), 3)
    yy = CubicSpline(x, y)
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
    for i in range(len(angles)):
        angles[i] = 90 - angles[i]

    ret = np.diff(angles)
    ret[0] = ret[0] - extra_angle
    return ret, distances

def draw_trajectory_left(xs, xf, phi, offset = 0, extra_angle = 0):
    x = [0, 10 - offset, 35 - offset, 80 - offset]
    y = [0, 15 - offset, 25 - offset, 35]
    xx = np.linspace(min(x), max(x), 8)
    yy = CubicSpline(x, y)
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
    for i in range(len(angles)):
        angles[i] = 90 - angles[i]

    ret = (-1) * np.diff(angles)
    ret[0] = ret[0] - extra_angle

    return ret, distances

def intersection_navigation(current, pipe, queuesList, offset = 0, extra_angle = 0, speed=15):
    if current == "RIGHT":
        # gostraight(pipe, queuesList, 0.5)
        angles, distances = draw_trajectory(0, 0, 0, offset, extra_angle)
        i = 0
        angle=0
        for dist in distances:
            t = dist / speed
            angle = angle + angles[i]
            steer(queuesList, angle)
            print(angle)
            i = i + 1
            time.sleep(t)
    elif current == "LEFT":
        gostraight(pipe, queuesList, 0.5)
        angles, distances = draw_trajectory_left(0, 0, 0, offset, extra_angle)
        i = 0
        angle=0
        for dist in distances:
            t = dist / speed
            angle = angle + angles[i]
            steer(queuesList, angle)
            print(angle)
            i = i + 1
            time.sleep(t)
    else:
        gostraight(pipe, queuesList, int(140/int(speed)))
