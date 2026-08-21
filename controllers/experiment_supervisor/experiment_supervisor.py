"""Stop a batch experiment after all UAVs have flushed their logs."""
from pathlib import Path
from controller import Supervisor

supervisor = Supervisor()
step = int(supervisor.getBasicTimeStep())
duration_file = Path(__file__).resolve().parents[2] / "experiment_duration.txt"
duration = float(duration_file.read_text().strip()) if duration_file.exists() else 3600.0
while supervisor.step(step) != -1:
    if supervisor.getTime() > duration + 1.0:
        supervisor.simulationQuit(0)
        break
