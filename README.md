# Charging-Constrained CBBA Experiment

Minimal Webots R2025a research implementation for four UAVs, eight recurring
construction-monitoring tasks, and three parallel charging ports.
The 1.4 m-wide pads have an assumed 1 m clear gap (2.4 m center spacing).

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
battery-feasibility constraint, 20% hard reserve, and pre-service energy check.

- `baseline`: standard CBBA plus route-energy feasibility. Charger availability
  is ignored during allocation, so conflicts are resolved at runtime.
- `proposed`: the baseline plus exclusive predicted charger intervals tied to
  a specific physical pad. The UAV
  with the larger energy margin accepts the earliest capacity- and
  energy-feasible interval, and truncates its bundle only when needed.

Task bundles and charger reservations are tracked independently. A UAV
requests a charger only when its predicted post-route energy cannot safely
support another task and return with the reserve intact. Charger reservations
are non-preemptable once committed. Future bundle tasks reopen to CBBA
competition whenever another available UAV observes a newly released task
generation; a task already in execution remains protected. Any allocation
change immediately invalidates and recomputes the dependent charging need and
reservation from the revised route.

Revisit deadlines apply to service starts. A task is released 15 s before its
next start is due, and route insertions that would make an achievable deadline
late are infeasible. A start after the due time is a violation. Any holding
time before a reserved charger interval is included in battery feasibility.
Each UAV starts at 90% SoC. Charging is progressive and linear from 20% to the
90% operational target in 40 minutes. Charging begins at touchdown. Charger
access explicitly includes descent to the pad, charging, takeoff, and ascent
back to cruise altitude. Reservations cover that entire sequence, and vertical
flight time and energy are included in feasibility calculations.
A UAV that crosses the 20% hard reserve is marked failed and removed from task
execution; it moves clear of the central structure or task prop when necessary,
then descends to make its out-of-service status visually unambiguous.

When a UAV has no executable bundle, it uses an unreserved charger gap only if
it provides at least 60 seconds of actual charging after travel, docking, and
departure time. Otherwise it lands at the nearest free original launch point
and takes off when a new task generation is released. A UAV performing a full
charge may also depart before 90% when a new task is released and its current
energy is sufficient to service a released task and return with the hard
reserve intact. If no energy-feasible charger interval is available, the UAV
waits landed and retries from the ground as slots clear; lack of immediate
charger access alone is not treated as a vehicle failure. A slot acquired from
the ground is checked against newly received peer state before takeoff, and
departure is delayed until travel to the reserved start time must begin. If the
assigned pad becomes conflicting before touchdown, overlapping committed slots
are ordered by start time and then UAV identifier. The losing UAV invalidates
its slot and returns to landed waiting instead of hovering at the dock.

All experimental constants and task definitions are in
`controllers/uav_cbba/mission_config.py`.

## Run

```sh
./run_experiment.sh
```

The script runs 120-minute baseline and proposed simulations with three
chargers, then writes `results/charger_count_metrics.csv`. Raw per-UAV
JSONL logs are stored beneath `results/chargers_3/` and ignored by Git.

To run one method interactively, set `CR_CBBA_MODE`, then
open `worlds/cr-cbba.wbt`. The world uses the available Mavic 2 Pro visual model;
the task-level battery parameters represent a DJI Mavic 3 Enterprise.

The current two-hour scenario uses 5-minute services, 15-minute safety
start-to-start revisits, and 25-minute quality start-to-start revisits.
