if __name__ == "__main__":
    import sys

    sys.path.insert(0, "../../..")

from src.templates.workerprocess import WorkerProcess
from src.kalman.threads.threadKalman import threadKalman
from multiprocessing import Pipe


class processKalman(WorkerProcess):

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processKalman, self).__init__(self.queuesList)

    # ===================================== STOP ==========================================
    def stop(self):
        """Function for stopping threads and the process."""
        super(processKalman, self).stop()

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads."""
        super(processKalman, self).run()

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Camera Publisher thread and add to the list of threads."""
        KalmanTh = threadKalman(self.queuesList, self.logging, self.debugging)
        self.threads.append(KalmanTh)