import time
from src.move.threads.movements.basic import setSpeed, steer
from src.move.threads.movements.PID import PID
from src.move.threads.movements.lane_following import followLane
import base64
import cv2
import numpy as np

kp = 1.2
kd = 0.65
ki = 0.0

distance_to_keep = 50

def tail(fdistpipe ,queuesList,imgpipe, positionpipe, coor_x, coor_y, K, t):
    print("Entered tailing")
    tailingPID = PID([kp,ki,kd],distance_to_keep,[-16,16])
    start_time = time.time()
    steer(queuesList,0)
    speed = 0
    angle = 0
    car_x_current = -10
    car_y_current = -10
    car_x_prev = -90
    car_y_prev = -90
    if positionpipe.poll():
        car_pos = positionpipe.recv()
        car_x_current = car_pos["value"]["x"]
        car_y_current = car_pos["value"]["y"]
        positionpipe.send("ready")

   
    counter = 0
    temp = time.time()
    while (True):      
        if fdistpipe.poll() and time.time()-temp > 0.2 :
            temp = time.time()
            dist = fdistpipe.recv()["value"]
            print(f"I am in tailing and i see distance: {dist}")
            fdistpipe.send("ready")
            if dist >= 70:
                print("The obstacle is gone!! I exit tailing!")
                return False
            speed = -tailingPID.update(dist)
            if(abs(angle)>15):
                speed = np.clip(speed, 0, 15)
            else:
                speed = np.clip(speed, 0, 20)
            setSpeed(queuesList, speed)
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

        if positionpipe.poll():
            car_pos = positionpipe.recv()
            car_x_prev = car_x_current
            car_y_prev = car_y_current 
            car_x_current = car_pos["value"]["x"]
            car_y_current = car_pos["value"]["y"]
            positionpipe.send("ready")
        
        if (abs((car_x_current-car_x_prev)+abs(car_y_current-car_y_prev))<20):
            counter+=1
            print("in tailing",counter)
            if(counter==5): 
                counter = 0
                return True
            #else:
            #    counter = 0
    
    print('Exited tailling!!!')
    return False


