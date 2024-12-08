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
import serial
from src.templates.workerprocess import WorkerProcess
from src.traffic_signs.threads.threadTraffic_Signs import threadTraffic_Signs
from src.traffic_signs.threads.threadPedestrian import threadPedestrian
from src.traffic_signs.threads.threadIndtersecDet import threadInterDet
from multiprocessing import Pipe
import time


class processTrafficSigns(WorkerProcess):
    """This process is the process that is in charge of the traffic signs detection (for now)
    Args:
        queueList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool, optional): A flag for debugging. Defaults to False.
    """

    # ===================================== INIT =========================================
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logger = logging
        self.debugging = debugging
        super(processTrafficSigns, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""
        super(processTrafficSigns, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processTrafficSigns, self).run()
        
    # ===================================== INIT TH =================================
    def _init_threads(self):
        """Initializes the read and the write thread."""
        signsTh = threadTraffic_Signs(self.queuesList, self.logger, self.debugging)
        self.threads.append(signsTh)
        pedTh = threadPedestrian(self.queuesList, self.logger, self.debugging)
        self.threads.append(pedTh)
        threadInDet = threadInterDet(self.queuesList, self.logger, self.debugging) 
        self.threads.append(threadInDet)