from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np

x_left = [0, 5, 20, 50, 80]
y_left = [0, 20, 45, 70, 88]

def draw_unparking_trajectory(offset, phi):
    x = [0, 10, 25, 40] 
    y = [0, 18, 35, 40]
    xx = np.linspace(min(x), max(x), 4)
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

    return np.diff(angles), distances
'''
def draw_parking_trajectory(phi, offset = 0):
    x = [15 - offset, 20 - offset, 95, 100 - offset]
    y = [19 - offset, 19 - offset, 45 - offset, 45 - offset]

    yy = CubicSpline(x, y)
    xx = np.linspace(min(x), max(x), 5)

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

    return np.diff(angles), distances

'''
