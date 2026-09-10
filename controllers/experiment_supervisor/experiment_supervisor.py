"""Stop the batch experiment after the UAV logs are flushed."""
import os
import sys
from pathlib import Path

from controller import Supervisor

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "uav_cbba"))
from mission_config import MISSION_DURATION_S  # noqa: E402

supervisor = Supervisor()
step = int(supervisor.getBasicTimeStep())
mission_duration = float(os.environ.get(
    "CR_CBBA_MISSION_DURATION", MISSION_DURATION_S))
while supervisor.step(step) != -1:
    if supervisor.getTime() > mission_duration + 1.0:
        supervisor.simulationQuit(0)
        break
