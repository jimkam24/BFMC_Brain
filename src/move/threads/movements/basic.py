from src.utils.messages.allMessages import (
SpeedMotor,
SteerMotor,
Brake,
Record,
CurrentSpeed,
FollowLane
)
import numpy as np

def enablelf(queuesList):
    queuesList[FollowLane.Queue.value].put( #send lane following flag
        {
            "Owner": FollowLane.Owner.value,
            "msgID": FollowLane.msgID.value,
            "msgType": FollowLane.msgType.value,
            "msgValue": True
        }   
    ) 

def disablelf(queuesList):
    queuesList[FollowLane.Queue.value].put( #send lane following flag
        {
            "Owner": FollowLane.Owner.value,
            "msgID": FollowLane.msgID.value,
            "msgType": FollowLane.msgType.value,
            "msgValue": False
        }   
    )    


def setSpeed(queuesList, speed=15):
    print("#----- setting speed -----#")
    print("speed = ", speed)
    queuesList[SpeedMotor.Queue.value].put(
    {
        "Owner": SpeedMotor.Owner.value,
        "msgID": SpeedMotor.msgID.value,
        "msgType": SpeedMotor.msgType.value,
        "msgValue": speed
    }
    )

    queuesList[CurrentSpeed.Queue.value].put( #send current velocity to do position calculation
        {
            "Owner": CurrentSpeed.Owner.value,
            "msgID": CurrentSpeed.msgID.value,
            "msgType": CurrentSpeed.msgType.value,
            "msgValue": float(speed)/100
        }   
    )

def steer(queuesList, angle):
    # print("#----- steering -----#")
    # print("angle =", angle)
    
    angle = np.clip(angle, -25, 25)

    queuesList[SteerMotor.Queue.value].put(
    {
        "Owner": SteerMotor.Owner.value,
        "msgID": SteerMotor.msgID.value,
        "msgType": SteerMotor.msgType.value,
        "msgValue": float(angle)
    }
    )

def brake(queuesList):
    print("#----- braking -----#")
    queuesList[Brake.Queue.value].put(
    {
        "Owner": Brake.Owner.value,
        "msgID": Brake.msgID.value,
        "msgType": Brake.msgType.value,
        "msgValue": 0
    }
    )

    queuesList[CurrentSpeed.Queue.value].put( #send current velocity to do position calculation
        {
            "Owner": CurrentSpeed.Owner.value,
            "msgID": CurrentSpeed.msgID.value,
            "msgType": CurrentSpeed.msgType.value,
            "msgValue": 0.0
        }   
    )

def start_recording(queuesList):
    queuesList[Record.Queue.value].put(
    {
        "Owner": Record.Owner.value,
        "msgID": Record.msgID.value,
        "msgType": Record.msgType.value,
        "msgValue": True
    }
    )
def stop_recording(queuesList):
    queuesList[Record.Queue.value].put(
    {
        "Owner": Record.Owner.value,
        "msgID": Record.msgID.value,
        "msgType": Record.msgType.value,
        "msgValue": False
    }
    )