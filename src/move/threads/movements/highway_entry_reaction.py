import time
from src.move.threads.movements.basic import setSpeed, steer, brake

def highway_entry(queuesList):
    setSpeed(queuesList,20)