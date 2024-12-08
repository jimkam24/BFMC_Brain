from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import sympy as smp
import math as ma

def draw_roundabout_trajectory(x, y, check, phi):
    C = 0.55
    radius = 65
    # points = np.array([
    # [C/2, -1-C/2], [3*C/4, -1], [C, -1], [1, -C], [1, 0], [1, C], 
    # [C, 1], [0, 1], 
    # [-C, 1], [-1, C], 
    # [-1, 0], [-1, -C], [-C, -1], [0, -1]
    # ]) * radius
    points = np.array([
    [0.1, -1.8], [0.5, -0.9], [0.8, -0.2], [0, 1],  [-0.8, 0], [0, -1]
    ]) * radius

    if(check == 1):
        points = np.array([
        [0.1, -1.8], [0.2, -1.6], [0.3, -1.4], [0.5, -0.9], [0.65, -0.5], [0.8, -0.3] ]) * radius
    elif(check == 2):
        points = np.array([
        [0.1, -1.8], [0.5, -0.9], [0.8, -0.2], [0.6, 0.4], [0.25, 0.8], ]) * radius
    elif(check == 3):
        points = np.array([
        [0.1, -1.8], [0.5, -0.9], [0.8, -0.2], [0, 1], [-0.4, 0.7], [-0.7, 0.2]]) * radius
    x, y = points.T
    # Creating a parametric spline. The 's' parameter controls the smoothness.
    # 'per' is set to True for a periodic spline (useful for closed paths)
    tck, u = splprep([x, y], s=1, per=False)

    # Generating new points along the spline for a smooth curve
    new_points = 1000
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
        

    return -1*np.diff(angles), distances

# draw_roundabout_trajectory(0,0,1,0)
# draw_roundabout_trajectory(0,0,2,0)
# draw_roundabout_trajectory(0,0,3,0)
# angles, dists = draw_parking_trajectory(0, 0, 0)
# print("distances", dists)
# print("angles", angles)