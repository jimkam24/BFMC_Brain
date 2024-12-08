import time
from src.move.threads.movements.basic import setSpeed, steer, brake

<<<<<<< HEAD


def pedestrian_reaction(queuesList,ped_seen):
    if ped_seen==True:
        brake(queuesList)
        time.sleep(3)
        
=======
def pedestrian_reaction(queuesList):
    print("seen pedestrian")
    brake(queuesList)
    time.sleep(2)
>>>>>>> 72fa1521fdb8afbbaa91a55b972aacaf77f43c84
