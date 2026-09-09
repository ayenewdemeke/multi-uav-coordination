#!/usr/bin/env python3
"""Aggregate the three-charger experiment into paper metrics."""
import csv
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "uav_cbba"))
from mission_config import (  # noqa: E402
    CHARGERS, MISSION_DURATION_S, RESERVE_FRACTION, TASKS as TASK_DEFINITIONS)

TASKS = {task[0]: (task[1], task[5]) for task in TASK_DEFINITIONS}
MODES = ("baseline", "proposed")
MISSION = MISSION_DURATION_S


def load(chargers, mode):
    rows = []
    folder = ROOT / "results" / f"chargers_{chargers}" / mode
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


def charging_seconds(rows):
    starts, total = {}, 0.0
    for row in rows:
        agent = row["agent"]
        if row["event"] == "dock_start":
            starts[agent] = row["time"]
        elif row["event"] == "dock_end" and agent in starts:
            total += row["time"] - starts.pop(agent)
    return total + sum(max(0.0, MISSION - start) for start in starts.values())


def summarize(chargers, mode, rows):
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
    cycles = [row["rounds"] for row in rows
              if row["event"] == "allocation_converged"]
    minimum_soc = min((row.get("minimum_soc", 1.0) for row in ends),
                      default=1.0)
    completed = {
        kind: sum(len(completions[task])
                  for task, (task_kind, _) in TASKS.items()
                  if task_kind == kind)
        for kind in ("Safety", "Quality")
    }
    used_seconds = charging_seconds(rows)
    failed = {row["agent"] for row in rows if row["event"] == "uav_failed"}
    return {
        "chargers": chargers,
        "mode": mode,
        "failed_uavs": len(failed),
        "charger_conflicts": sum(row["event"] == "charger_conflict"
                                 for row in rows),
        "charger_waiting_s": round(sum(
            row.get("duration_s", 0.0) for row in rows
            if row["event"] == "charger_wait_end"), 3),
        "safety_revisit_violations": violations["Safety"],
        "quality_revisit_violations": violations["Quality"],
        "total_revisit_violations": sum(violations.values()),
        "cumulative_start_lateness_s": round(sum(
            max(0.0, row["time"] - row["deadline"]) for row in rows
            if row["event"] == "task_start"), 3),
        "completed_safety_services": completed["Safety"],
        "completed_quality_services": completed["Quality"],
        "completed_services": sum(completed.values()),
        "reserve_violations": sum(
            row.get("minimum_soc", 1.0) < RESERVE_FRACTION for row in ends),
        "minimum_soc_pct": round(100.0 * minimum_soc, 2),
        "charging_events": sum(row["event"] == "charge_start" for row in rows),
        "charger_utilization_pct": round(
            100.0 * used_seconds / (chargers * MISSION), 3),
        "charging_reassignments": sum(
            row.get("released_count", 1) for row in rows
            if row["event"] == "charger_conflict_resolved"),
        "reservation_shifts": sum(
            row["event"] == "charger_slot_shifted" for row in rows),
        "mean_allocation_rounds": round(
            sum(cycles) / len(cycles), 3) if cycles else 0.0,
        "communication_messages": sum(row.get("messages", 0) for row in ends),
    }


def main():
    chargers = len(CHARGERS)
    data = {mode: load(chargers, mode) for mode in MODES}
    missing = [mode for mode, rows in data.items() if not rows]
    if missing:
        raise SystemExit("Missing logs for: " + ", ".join(missing))
    summaries = [summarize(chargers, mode, data[mode]) for mode in MODES]
    destination = ROOT / "results" / "charger_count_metrics.csv"
    with destination.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)
    for row in summaries:
        print(json.dumps(row, indent=2))


if __name__ == "__main__":
    main()
