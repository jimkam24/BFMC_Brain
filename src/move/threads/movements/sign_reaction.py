import time
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.parking_reaction import parking_reaction, parking_reaction_left

def sign_reaction(queuesList, sign, pipe = None, offset = 0, pos = 'RIGHT'):
    if sign == "Stop":
        brake(queuesList)
        time.sleep(1)
        steer(queuesList,0)
        time.sleep(2)

    elif sign == "roundabout":
        time.sleep(3)

    elif sign == "Parking":
        if (pos == 'RIGHT'):
            parking_reaction(queuesList, offset, pipe)
        elif(pos == 'LEFT'):
            parking_reaction_left(queuesList, offset, pipe)
    
    elif sign == "highway_entry":
        setSpeed(queuesList,25) 
    
    elif sign == "highway_exit":
        setSpeed(queuesList,16)
    
    elif sign == "Crosswalk":
        while (pipe.poll()):
            Pedestrian =pipe.recv()["value"]
            pipe.send("ready")
            print("seen pedestrian")
            brake(queuesList)
            time.sleep(2)
        
    elif sign == "Priority":
        steer(queuesList,0)

    

    
    
    
            
