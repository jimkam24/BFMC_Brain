import time
from src.move.threads.movements.basic import setSpeed, steer, brake

def highway_exit(queuesList):
    setSpeed(queuesList,10)