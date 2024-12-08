from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
import time

def Semaphores_reaction(id,state,queuesList,speed):
    if ((id in range(0,5)) and (state == "red" or state == "yellow")):
        brake(queuesList)
        time.sleep(1)
        steer(queuesList,0)
    
    if ((id in range(0,5)) and state == "green"):
        # setSpeed(queuesList,speed)
        pass



