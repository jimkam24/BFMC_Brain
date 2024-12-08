import time
from src.move.threads.movements.basic import setSpeed, steer, brake



def crosswalk_reaction(queuesList,ped_seen):
    brake(queuesList)
    time.sleep(3)
    
