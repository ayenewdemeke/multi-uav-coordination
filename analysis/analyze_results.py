#!/usr/bin/env python3
"""Aggregate the charger-count ablation into paper metrics."""
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "uav_cbba"))
from mission_config import (  # noqa: E402
    CHARGERS, MISSION_DURATION_S, TASKS as TASK_DEFINITIONS)

TASKS = {task[0]: (task[1], task[5]) for task in TASK_DEFINITIONS}
MISSION = MISSION_DURATION_S


METHODS = ("proposed", "battery_only")


FLEET_SIZES = (3, 4, 5, 6)
CHARGER_COUNTS = (1, 2, 3, 4)


def load(method, uavs, chargers):
    rows = []
    folder = ROOT / "results" / method / f"uav{uavs}_chargers_{chargers}"
    for path in sorted(folder.glob("UAV*.jsonl")):
        rows.extend(json.loads(line) for line in path.read_text().splitlines()
                    if line)
    return sorted(rows, key=lambda row: (row["time"], row["agent"]))


def task_events(rows, event):
    result = {task: [] for task in TASKS}
    for row in rows:
        if row["event"] == event:
            result[row["task"]].append(row["time"])
    return result


# A dock attempt that finds the pad taken aborts without a dock_end.  Closing
# those at the mission horizon counts an aborted approach as a full occupancy
# and can push reported utilization above 100%.
ABORTED_DOCK = {"charger_conflict", "charger_deferred_to_ground",
                "standby_landed", "charger_request", "uav_failed"}


def charging_seconds(rows):
    starts, total = {}, 0.0
    for row in rows:
        agent = row["agent"]
        if row["event"] == "dock_start":
            starts[agent] = row["time"]
        elif agent in starts and (row["event"] == "dock_end" or
                                  row["event"] in ABORTED_DOCK):
            total += row["time"] - starts.pop(agent)
    return total + sum(max(0.0, MISSION - start) for start in starts.values())


def summarize(method, uavs, chargers, rows):
    starts = task_events(rows, "task_start")
    completions = task_events(rows, "task_complete")
    violations = {"Safety": 0, "Quality": 0}
    for task, (kind, revisit) in TASKS.items():
        previous = 0.0
        for current in starts[task] + [MISSION]:
            violations[kind] += max(
                0, math.ceil((current - previous) / revisit) - 1)
            previous = current
    ends = [row for row in rows if row["event"] == "mission_end"]
    if len(ends) != uavs:
        raise ValueError(
            f"Incomplete {method} uav{uavs}/ch{chargers} run: "
            f"{len(ends)}/{uavs} UAVs reached mission end")
    cycles = [row["rounds"] for row in rows
              if row["event"] == "allocation_converged" and
              not row.get("forced", False)]
    completed = {
        kind: sum(len(completions[task])
                  for task, (task_kind, _) in TASKS.items()
                  if task_kind == kind)
        for kind in ("Safety", "Quality")
    }
    used_seconds = charging_seconds(rows)
    failed = {row["agent"] for row in rows if row["event"] == "uav_failed"}
    # Lateness is reported per service start.  The unnormalized sum rewards
    # configurations that serve fewer tasks, because a task that is never
    # started contributes no lateness at all.
    lateness = [max(0.0, row["time"] - row["deadline"]) for row in rows
                if row["event"] == "task_start"]
    return {
        "method": method,
        "uavs": uavs,
        "chargers": chargers,
        "failed_uavs": len(failed),
        # Constraint (5) is enforced during allocation, so a physical pad
        # collision at execution time is a violation, not a routine event.
        # A UAV that wanted a pad and none was available, or that flew to one
        # and found it occupied.  Dropping an uncommitted proposal during
        # consensus is renegotiation, not a denial of access, and is reported
        # separately.
        "access_conflicts": sum(
            row["event"] in ("charger_conflict",
                             "charger_deferred_to_ground")
            for row in rows),
        "charger_waiting_s": round(sum(
            row.get("duration_s", 0.0) for row in rows
            if row["event"] == "charger_wait_end"), 3),
        "safety_revisit_violations": violations["Safety"],
        "quality_revisit_violations": violations["Quality"],
        "total_revisit_violations": sum(violations.values()),
        "mean_start_lateness_s": round(
            sum(lateness) / len(lateness), 3) if lateness else 0.0,
        "late_start_fraction": round(
            sum(value > 0.0 for value in lateness) / len(lateness),
            4) if lateness else 0.0,
        "cumulative_start_lateness_s": round(sum(lateness), 3),
        "completed_safety_services": completed["Safety"],
        "completed_quality_services": completed["Quality"],
        "completed_services": sum(completed.values()),
        "charging_events": sum(row["event"] == "charge_start" for row in rows),
        "charger_utilization_pct": round(
            100.0 * used_seconds / (chargers * MISSION), 3),
        "slot_renegotiations": sum(
            row["event"] in ("charger_slot_shifted",
                             "ground_charger_slot_invalidated")
            for row in rows),
        "reservation_shifts": sum(
            row["event"] == "charger_slot_shifted" for row in rows),
        "forced_auction_exits": sum(
            row["event"] == "allocation_converged" and row.get("forced", False)
            for row in rows),
        "mean_converged_allocation_rounds": round(
            sum(cycles) / len(cycles), 3) if cycles else 0.0,
        "communication_messages": sum(row.get("messages", 0) for row in ends),
    }


def main():
    cells = [(m, u, c) for m in METHODS
             for u in FLEET_SIZES for c in CHARGER_COUNTS]
    data = {cell: load(*cell) for cell in cells}
    present = [cell for cell in cells if data[cell]]
    if not present:
        raise SystemExit("No result logs found under results/")
    summaries = [summarize(m, u, c, data[m, u, c]) for m, u, c in present]
    destination = ROOT / "results" / "charger_count_metrics.csv"
    with destination.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)
    for row in summaries:
        print(json.dumps(row, indent=2))


if __name__ == "__main__":
    main()
