"""Stop the 120-minute batch experiment after the UAV logs are flushed."""
from controller import Supervisor

supervisor = Supervisor()
step = int(supervisor.getBasicTimeStep())
while supervisor.step(step) != -1:
    if supervisor.getTime() > 7201.0:
        supervisor.simulationQuit(0)
        break
