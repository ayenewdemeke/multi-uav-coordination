# Charging-Constrained CBBA Experiment

This Webots R2025a project implements the experiment in *Charging-Constrained
Decentralized Task Allocation for Persistent Multi-UAV Construction
Monitoring*: four Mavic 2 Pro UAVs, four recurring safety tasks (`S1`–`S4`),
four recurring quality tasks (`Q1`–`Q4`), and one unit-capacity charger.

## Matched treatments

- `baseline`: energy-feasible CBBA ignores charger availability during
  allocation; simultaneous arrivals wait during execution.
- `proposed`: the same scoring and energy model includes exclusive predicted
  30 s charger reservations. On a conflict, the UAV with more remaining energy
  margin truncates its bundle and releases a task for reassignment.

Both treatments use 213,444 J capacity, 129 W consumption, 4 m/s planning
speed, a 15% reserve, and Table I's priorities, revisit intervals, and service
durations. Clustered initial charge levels create the controlled contention
scenario specified in the paper.

## Running the experiment

The controller defaults to `proposed`. Run both treatments and analysis with:

```sh
./run_experiment.sh
```

Or run them separately:

```sh
CR_CBBA_MODE=baseline webots --batch --mode=fast worlds/cr-cbba.wbt
CR_CBBA_MODE=proposed webots --batch --mode=fast worlds/cr-cbba.wbt
python3 analysis/analyze_results.py
```

Per-UAV event logs go to `results/<mode>/`. Analysis creates the Table II CSV,
the charger-access timeline (Fig. 2), and overdue-task plot (Fig. 3).
Matplotlib is optional; without it, the metrics CSV is still generated.

The manuscript's current Results numbers are illustrative placeholders. Replace
them only after running and reviewing the generated results.

## Generated 60-minute results

The initial run produced 2 charger conflicts and 26.184 s of
waiting for the baseline, versus 0 and 0 s for the proposed method. The
proposed method completed 23 services versus 16, reduced missed safety/quality
revisit deadlines from 91/26 to 88/24, and made five charging-induced task
reassignments. Minimum SoC was 21.19% and 22.39%, respectively. These outputs
are preserved under `results/initial_run/`.

The high-contention run uses identical 55% initial SoC for all UAVs. It
produced 5 baseline charger conflicts and 82.976 s of waiting, versus 0 and 0 s
for the proposed method, with 11 charging-induced reassignments. Safety misses
decreased from 91 to 89, while quality misses increased from 23 to 26 and
completed services decreased from 21 to 18. This exposes the coverage cost of
aggressive conflict avoidance and is preserved under
`results/high_contention/`. The root `results/` outputs also refer to this
latest scenario.

A revisit violation is
counted for every prescribed revisit deadline missed, including multiple
deadlines within a long unobserved interval.

## Tests

```sh
python3 -m unittest discover -s tests -v
python3 -m py_compile controllers/uav_cbba/*.py analysis/analyze_results.py
```
