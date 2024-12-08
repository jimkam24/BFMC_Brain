import time

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording

def stop_reaction(queuesList):
    brake(queuesList)
    time.sleep(3)

