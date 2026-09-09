"""Stop the batch experiment after the UAV logs are flushed."""
import os

from controller import Supervisor

supervisor = Supervisor()
step = int(supervisor.getBasicTimeStep())
mission_duration = float(os.environ.get("CR_CBBA_MISSION_DURATION", "28800"))
while supervisor.step(step) != -1:
    if supervisor.getTime() > mission_duration + 1.0:
        supervisor.simulationQuit(0)
        break
