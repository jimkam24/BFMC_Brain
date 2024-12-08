import time
from src.move.threads.movements.basic import setSpeed, steer
from src.move.threads.movements.PID import PID
from src.move.threads.movements.lane_following import followLane
import base64
import cv2
import numpy as np

kp_dist = 0.5
ki_dist = 0.01
kd_dist = 0.1

kp_speed = 1.0
ki_speed = 0.1
kd_speed = 0.01

distance_to_keep = 30
des_speed = 15

def tail(fdistpipe, queuesList, imgpipe, K, t):
    dist_PID = PID([kp_dist, ki_dist, kd_dist], distance_to_keep, [-100, 100])
    speed_PID = PID([kp_speed, ki_speed, kd_speed], des_speed, [-20, 20])

    start_time = time.time()
    speed = 15

    while(time.time() - start_time <= t):
        if fdistpipe.poll():
            dist = fdistpipe.recv()["value"]
            print(dist)
            fdistpipe.send("ready")

            speed_target = speed_PID.update(dist)
            speed_PID.update_target(speed_target)
            speed_control = speed_PID.update(speed)
            speed += speed_control
            print("speed: " + str(speed))
            setSpeed(queuesList, speed)

        if imgpipe.poll():
            frame = imgpipe.recv()
            image_data = base64.b64decode(frame["value"])
            img = np.frombuffer(image_data, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            angle, offset = followLane(img,K, speed)
            steer(queuesList, angle)
