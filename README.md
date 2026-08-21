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

Bundles contain at most two tasks. Both treatments apply the same task score,
battery-feasibility constraint, 15% reserve, and pre-service energy check.

- `baseline`: standard CBBA plus route-energy feasibility. Charger availability
  is ignored during allocation, so conflicts are resolved by waiting at runtime.
- `proposed`: the baseline plus exclusive predicted charger intervals. An
  overlapping interval causes the UAV with more energy margin to truncate its
  bundle, releasing a task for normal CBBA reassignment.

The reservation agreed at convergence remains attached to the route through
execution: the UAV completes its committed bundle, returns to the charger, and
uses that interval without recomputing it. Winner, bid, and timestamp vectors
continue to be exchanged in every flight state, while status messages expose
active and committed tasks to later allocation cycles. A committed charger
reservation is non-preemptable until used; tentative bundles remain open to
normal CBBA competition and can still be outbid before convergence.

After a task is completed, its next revisit has a distinct release time and
deadline. The release lead is derived from its service duration and the longest
possible inbound site trip, allowing an on-time completion instead of waiting
until the deadline has already passed. Any holding time before a reserved
charger interval is included in battery feasibility.

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
