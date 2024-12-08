# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
from enum import Enum


####################################### processCamera #######################################
class mainCamera(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 1
    msgType = "base64"


class serialCamera(Enum):
    Queue = "Critical"
    Owner = "threadCamera"
    msgID = 2
    msgType = "base64"


class Recording(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 3
    msgType = "Boolean"


class Signal(Enum):
    Queue = "General"
    Owner = "threadCamera"
    msgID = 4
    msgType = "String"


################################# processCarsAndSemaphores ##################################
class Cars(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 1
    msgType = "String"


class Semaphores(Enum):
    Queue = "General"
    Owner = "threadCarsAndSemaphores"
    msgID = 2
    msgType = "String"


################################# From Move ##################################
class EngineRun(Enum):
    Queue = "General"
    Owner = "threadMove"
    msgID = 1
    msgType = "dictionary"


# {"action": "startEngine", "value": self.started}


class SpeedMotor(Enum):
    Queue = "General"
    Owner = "threadMove"
    msgID = 2
    msgType = "dictionary"


# "action": "speed", "value": val}


class SteerMotor(Enum):
    Queue = "Critical"
    Owner = "threadMove"
    msgID = 3
    msgType = "dictionary"


# {"action": "steer", "value": val}


class Control(Enum):
    Queue = "General"
    Owner = "threadRemoteHandler"
    msgID = 4
    msgType = "dictionary"


class Brake(Enum):
    Queue = "Critical"
    Owner = "threadMove"
    msgID = 5
    msgType = "dictionary"


# {"action": "steer", "value": 0.0}
# {"action": "speed", "value": 0.0}


class Record(Enum):
    Queue = "General"
    Owner = "threadMove"
    msgID = 6
    msgType = "dictionary"


# {"action": "startRecord", "value": self.startedRecord}


class Config(Enum):
    Queue = "General"
    Owner = "threadRemoteHandler"
    msgID = 7
    msgType = "dictionary"


# {"action": key, "value": value}

class LaneError(Enum):
    Queue = "General"
    Owner = "threadMove"
    msgID = 8
    msgType = "dictionary"
    
class FollowLane(Enum):
    Queue = "Critical"
    Owner = "threadMove"
    msgID = 9
    msgType = "bool"

################################# From Remote Handler ##################################

class MoveConfig(Enum):
    Queue = "General"
    Owner = "threadRemoteHandler"
    msgID = 1
    msgType = "dictionary"


# {"action": key, "value": value}
    
#==========================FOR PATH PLANNING=======================

class Calculate(Enum):
    Queue = "General"
    Owner = "threadMove"
    msgID = 4
    msgType = "dictionary"


class Path(Enum):
    Queue = "General"
    Owner = "threadPathPlanning"
    msgID = 9
    msgType = "dictionary"

#=========================FOR INTERSECTION DET========================
    
class Estimate(Enum):
    Queue = "General"
    Owner = "theadMove"
    msgID = 10
    msgType = "dictionary"

class InterDistance(Enum):
    Queue = "General"
    Owner = "threadInterDet"
    msgID = 11
    msgType = "dictionary"

#=========================FOR KALMAN FILTER========================
    
class CurrentSpeed(Enum):
    Queue = "General"
    Owner = "theadMove"
    msgID = 12
    msgType = "dictionary"

class Pos(Enum):
    Queue = "General"
    Owner = "threadKalman"
    msgID = 13
    msgType = "dictionary"

################################# From Nucleo ##################################
class BatteryLvl(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 1
    msgType = "float"


class ImuData(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 2
    msgType = "String"


class InstantConsumption(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 3
    msgType = "float"

class FrontDistance(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 4
    msgType = "float"

class LeftDistance(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 5
    msgType = "float"
    
class RightDistance(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 6
    msgType = "float"

class Infrared(Enum):
    Queue = "Critical"
    Owner = "threadRead"
    msgID = 7
    msgType = "float"

class LeftDistance(Enum):
    Queue = "General"
    Owner = "threadRead"
    msgID = 8
    msgType = "float"


################################# From Locsys ##################################
class Location(Enum):
    Queue = "General"
    Owner = "threadTrafficCommunication"
    msgID = 1
    msgType = "dictionary"


# {"x": value, "y": value}


######################    From processSerialHandler  ###########################
class EnableButton(Enum):
    Queue = "General"
    Owner = "threadWrite"
    msgID = 1
    msgType = "Boolean"


class SignalRunning(Enum):
    Queue = "General"
    Owner = "threadWrite"
    msgID = 2
    msgType = "Boolean"
    

################################# From Vision ##################################

class TrafficSign(Enum):
    Queue = "General"
    Owner = "threadTraffic_Signs"
    msgID = 1
    msgType = "string"

class Pedestrian(Enum):
    Queue = "General"
    Owner = "threadPedestrian"
    msgID = 1
    msgType = "boolean"

class SignsSearching(Enum):
    Queue = "General"
    Owner = "threadMove"
    msgID = 14
    msgType = "dictionary"