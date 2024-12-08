# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ===================================== GENERAL IMPORTS ==================================
import sys
import time

sys.path.append(".")
from multiprocessing import Queue, Event
import logging


# ===================================== PROCESS IMPORTS ==================================
from src.gateway.processGateway import processGateway
from src.hardware.camera.processCamera import processCamera
from src.hardware.serialhandler.processSerialHandler import processSerialHandler
from src.utils.PCcommunicationDemo.processPCcommunication import (
    processPCCommunicationDemo,
)
from src.utils.PCcommunicationDashBoard.processPCcommunication import (
    processPCCommunicationDashBoard,
)
from src.move.processMove_fast import processMove

# ======================================== SETTING UP ====================================
allProcesses = list()
queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue(),
}

logging = logging.getLogger()

Camera = True
PCCommunicationDemo = False
PCCommunicationDashBoard = True
SerialHandler = True
Move = True
# ===================================== SETUP PROCESSES ==================================

# Initializing gateway
processGateway = processGateway(queueList, logging)
allProcesses.append(processGateway)

# Initializing camera
if Camera:
    processCamera = processCamera(queueList, logging)
    allProcesses.append(processCamera)

# Initializing interface
if PCCommunicationDemo:
    processPCCommunication = processPCCommunicationDemo(queueList, logging)
    allProcesses.append(processPCCommunication)
elif PCCommunicationDashBoard:
    processPCCommunicationDashBoard = processPCCommunicationDashBoard(
        queueList, logging
    )
    allProcesses.append(processPCCommunicationDashBoard)

# Initializing serial connection NUCLEO --> PI
if SerialHandler:
    processSerialHandler = processSerialHandler(queueList, logging)
    allProcesses.append(processSerialHandler)

# Initializing the process for decision making
if Move:
    processMove = processMove(queueList, logging)
    allProcesses.append(processMove)



# ===================================== START PROCESSES ==================================
for process in allProcesses:
    process.daemon = True
    process.start()

# ===================================== STAYING ALIVE ====================================
blocker = Event()
try:
    blocker.wait()
except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
    allProcesses.reverse()
    for proc in allProcesses:
        print("Process stopped", proc)
        proc.stop()
        # proc.join()
        proc.join(0.1)
        if proc.is_alive:
            proc.terminate()
            
# temporary solution (maybe) -> 
# the code they have provided does not work as expected with stop for all processes, so we 
# use terminate for the ones that do not stop:
#
# can this cause problems??
# is there a better solution?? 