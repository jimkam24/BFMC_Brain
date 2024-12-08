from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt

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





# a, d = draw_parking_trajectory(0, 0, 0)

# print(a)
# print(d)
