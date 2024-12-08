import numpy as np
from numpy.linalg import inv

def predict(x, F, Rot, v, P, Q, dt):
    x = F@x + (Rot@v)*dt
    P = F@P@F.T + Q

    return x, P


def update(x, z, H, P, R):
    y = z - H@x
    S = H@P@H.T + R
    K = P@H.T@inv(S)
    x = x + K@y
    P = (np.eye(2, 2) - K@H)@P

    return x, P
