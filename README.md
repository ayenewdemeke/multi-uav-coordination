# Charging-Constrained CBBA Experiment

Minimal Webots R2025a research implementation for four UAVs, eight recurring
construction-monitoring tasks, and one unit-capacity charger.

UAV motion is kinematic at the configured 4 m/s. This matches the manuscript's
constant-speed, constant-power task-level model and keeps predicted task and
charger arrival times consistent with execution.

## Methods

Both treatments use the standard two-phase Consensus-Based Bundle Algorithm:

1. sequential greedy bundle construction using marginal insertion bids;
2. the timestamp-based CBBA consensus rules of Choi, Brunet, and How (2009),
   including winner/bid vectors and bundle truncation after a lost task.

Bundles may contain up to all eight tasks; route energy, ownership, and timing
determine their actual length. Both treatments apply the same task score,
battery-feasibility constraint, 15% reserve, and pre-service energy check.

- `baseline`: standard CBBA plus route-energy feasibility. Charger availability
  is ignored during allocation, so conflicts are resolved by waiting at runtime;
  a worst-case three-slot energy allowance protects the 15% reserve.
- `proposed`: the baseline plus exclusive predicted charger intervals. The UAV
  with the larger energy margin accepts an energy-feasible slot delayed by at
  most 60 s and truncates its bundle only when none is available.

Task bundles and charger reservations are committed independently. A UAV
requests a charger only when its predicted post-route energy cannot safely
support another task and return with the reserve intact. A committed charger
reservation is non-preemptable until used; tentative bundles remain open to
normal CBBA competition.

Revisit deadlines apply to service starts. A task is released 15 s before its
next start is due, and route insertions that would make an achievable deadline
late are infeasible. A start after the due time is a violation. Any holding
time before a reserved charger interval is included in battery feasibility.
Battery replenishment occurs at docking, while the charger remains occupied for
the full 30 s access interval.

All experimental constants and task definitions are in
`controllers/uav_cbba/mission_config.py`.

## Run

```sh
./run_experiment.sh
```

The script runs separate 120-minute baseline and proposed simulations, then
writes the comparison CSV and figures under `results/`. Raw per-UAV JSONL logs
are ignored by Git. Plotting requires matplotlib; the CSV does not.

To run one method interactively, put `baseline` or `proposed` in
`experiment_mode.txt` and open `worlds/cr-cbba.wbt`.
