import time
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.parking import draw_parking_trajectory
from src.move.threads.movements.intersection import find_targ
from src.move.threads.movements.PID import PID
from src.move.threads.movements.unparking import draw_unparking_trajectory

def parking_reaction(queuesList, offset, pipe):
    # parking
    a, d = draw_parking_trajectory(0, offset)
    i = 0
    angle = 0
    setSpeed(queuesList, -16)
    for dist in d:
        t = dist / 15.0
        if a[i] <= 0 and angle > 0:
            angle = -12
        angle = angle + a[i]
        if angle > 23:
            angle = 23
        elif angle < -23:
            angle = -23
        steer(queuesList, angle)
        i = i + 1
        time.sleep(t)

    brake(queuesList)
    steer(queuesList, 0)
    time.sleep(1)
    print("now correcting position")
    
    # correct your parking position
    line = 0
    if (pipe[0].poll()):
        dummy = pipe[0].recv()
    pipe[0].send("ready")
    while (not pipe[0].poll()):
        continue
    angle = float(pipe[0].recv()["value"]["yaw"])
    target = find_targ(angle, "STRAIGHT")
    pipe[0].send("ready")
    pid = PID((4, 0.0, 0.05), target, [-15, 15])
    setSpeed(queuesList, 10)
    while(True):
        if pipe[0].poll():
            data = pipe[0].recv()["value"]
            if (float(data["yaw"]) > 315):
                angle = float(data["yaw"]) - 360
            else:
                angle = float(data["yaw"])
            print("angle", angle)
            pipe[0].send("ready")
            steering_angle = pid.update(angle)
            print("steer", steering_angle)
            steer(queuesList, steering_angle)
        if pipe[1].poll():
            if (pipe[1].recv()["value"] == 1):
                line = 1
                break
            pipe[1].send("ready")
    brake(queuesList)
    time.sleep(1)
    
    print("corrected position")
    
    if line:
        print("line")
        steer(queuesList, 0)
        setSpeed(queuesList, -10)
        t0 = time.time()
        while((time.time() - t0 <= 2)):
            if pipe[0].poll():
                data = pipe[0].recv()["value"]
                if (float(data["yaw"]) > 315):
                    angle = float(data["yaw"]) - 360
                else:
                    angle = float(data["yaw"])
                print("angle", angle)
                pipe[0].send("ready")
                steering_angle = pid.update(angle)
                print("steer", steering_angle)
                steer(queuesList, -steering_angle)
    brake(queuesList)
    steer(queuesList, 0)
    time.sleep(5)
    
    #unparking
    a, d = draw_unparking_trajectory(0, 0)
    i = 0
    angle = 0
    setSpeed(queuesList, -10)
    time.sleep(1)
    setSpeed(queuesList, 16)
    for dist in d:
        t = dist / 15.0
        angle -= a[i]
        if angle > 23:
            angle = 23
        elif angle < -23:
            angle = -23
        steer(queuesList, -angle)
        i = i + 1
        time.sleep(t)
    brake(queuesList)
    setSpeed(queuesList, 16)
    steer(queuesList, 23)
    time.sleep(1)

def parking_reaction_left(queuesList, offset, pipe):
    # parking
    a, d = draw_parking_trajectory(0, offset)
    i = 0
    angle = 0
    setSpeed(queuesList, -16)
    for dist in d:
        t = dist / 15.0
        if -a[i] >= 0 and angle < 0:
            angle = +12
        angle -= a[i]
        if angle > 23:
            angle = 23
        elif angle < -23:
            angle = -23
        steer(queuesList, angle)
        i = i + 1
        time.sleep(t)

    brake(queuesList)
    steer(queuesList, 0)
    time.sleep(1)
    print("now correcting position")
    
    # correct your parking position
    line = 0
    if (pipe[0].poll()):
        dummy = pipe[0].recv()
    pipe[0].send("ready")
    while (not pipe[0].poll()):
        continue
    angle = float(pipe[0].recv()["value"]["yaw"])
    target = find_targ(angle, "STRAIGHT")
    pipe[0].send("ready")
    pid = PID((4, 0.0, 0.05), target, [-15, 15])
    setSpeed(queuesList, 10)
    while(True):
        if pipe[0].poll():
            data = pipe[0].recv()["value"]
            if (float(data["yaw"]) > 315):
                angle = float(data["yaw"]) - 360
            else:
                angle = float(data["yaw"])
            print("angle", angle)
            pipe[0].send("ready")
            steering_angle = pid.update(angle)
            print("steer", steering_angle)
            steer(queuesList, steering_angle)
        if pipe[1].poll():
            if (pipe[1].recv()["value"] == 1):
                line = 1
                break
            pipe[1].send("ready")
    brake(queuesList)
    time.sleep(1)
    
    print("corrected position")
    
    if line:
        print("line")
        steer(queuesList, 0)
        setSpeed(queuesList, -10)
        t0 = time.time()
        while((time.time() - t0 <= 2)):
            if pipe[0].poll():
                data = pipe[0].recv()["value"]
                if (float(data["yaw"]) > 315):
                    angle = float(data["yaw"]) - 360
                else:
                    angle = float(data["yaw"])
                print("angle", angle)
                pipe[0].send("ready")
                steering_angle = pid.update(angle)
                print("steer", steering_angle)
                steer(queuesList, -steering_angle)
    brake(queuesList)
    steer(queuesList, 0)
    time.sleep(5)
    
    #unparking
    a, d = draw_parking_trajectory(0, offset)
    i = 0
    angle = 0
    setSpeed(queuesList, -10)
    time.sleep(1)
    setSpeed(queuesList, 16)
    for dist in d:
        t = dist / 15.0
        if a[i] <= 0 and angle > 0:
            angle = 15
        angle -= a[i]
        if angle > 23:
            angle = 23
        elif angle < -23:
            angle = -23
        steer(queuesList, -angle)
        i = i + 1
        time.sleep(t)
