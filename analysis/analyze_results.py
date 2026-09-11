#!/usr/bin/env python3
"""Aggregate the fleet-size and charger-count ablation into paper metrics.

Every result set is checked for physical consistency before any metric is
reported; a failure here means the run is unusable, not that a metric is low.
"""
import csv
import itertools
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "uav_cbba"))
from mission_config import (  # noqa: E402
    CHARGERS, MISSION_DURATION_S, ROSTER, TASKS as TASK_DEFINITIONS)

KIND = {task[0]: task[1] for task in TASK_DEFINITIONS}
METHODS = ("proposed", "battery_only")
FLEET_SIZES = (3, 4, 5, 6)
CHARGER_COUNTS = tuple(range(1, len(CHARGERS) + 1))

# A dock attempt that finds the resource taken aborts without a dock_end.
ABORTED_DOCK = {"charger_conflict", "charger_deferred_to_ground",
                "standby_landed", "charger_request", "uav_failed"}


def load(method, uavs, chargers):
    folder = ROOT / "results" / method / f"uav{uavs}_chargers_{chargers}"
    rows = []
    for path in sorted(folder.glob("UAV*.jsonl")):
        rows.extend(json.loads(line) for line in path.read_text().splitlines()
                    if line.strip())
    return sorted(rows, key=lambda row: (row["time"], row["agent"]))


def dock_sessions(rows, horizon):
    """(resource, start, end, agent) for every completed occupancy."""
    sessions, open_docks = [], {}
    for row in rows:
        agent, event = row["agent"], row["event"]
        if event == "dock_start":
            open_docks[agent] = (row["time"], row.get("charger_index"))
        elif agent in open_docks and (event == "dock_end"
                                      or event in ABORTED_DOCK):
            start, pad = open_docks.pop(agent)
            if row["time"] - start > 0.05:
                sessions.append((pad, start, row["time"], agent))
    for agent, (start, pad) in open_docks.items():
        sessions.append((pad, start, horizon, agent))
    return sessions


def check(method, uavs, chargers, rows, horizon):
    """Physical-consistency gate.  Returns a list of violations."""
    tag = f"{method} uav{uavs}/ch{chargers}"
    bad = []
    agents = {row["agent"] for row in rows}
    if len(agents) != uavs:
        bad.append(f"{tag}: {len(agents)} UAVs logged, expected {uavs}")
    ends = [r for r in rows if r["event"] == "mission_end"]
    if len(ends) != uavs:
        bad.append(f"{tag}: {len(ends)}/{uavs} UAVs reached mission end")
    sessions = dock_sessions(rows, horizon)
    for a, b in itertools.combinations(sessions, 2):
        if a[0] == b[0] and a[1] < b[2] and b[1] < a[2]:
            bad.append(f"{tag}: resource {a[0]} held by {a[3]} and {b[3]} "
                       f"simultaneously")
    for pad in {p for p, _, _, _ in sessions}:
        busy = sum(e - s for p, s, e, _ in sessions if p == pad)
        if busy > horizon + 1.0:
            bad.append(f"{tag}: resource {pad} occupied beyond the horizon")
    for row in rows:
        for key in ("battery_soc", "arrival_soc", "departure_soc"):
            if key in row and row[key] < 0.20 - 1e-6:
                bad.append(f"{tag}: {row['agent']} {key}={row[key]:.3f} "
                           f"below the prescribed reserve")
    starts = sum(1 for r in rows if r["event"] == "task_start")
    completes = sum(1 for r in rows if r["event"] == "task_complete")
    if completes > starts:
        bad.append(f"{tag}: {completes} completions from {starts} starts")
    return bad


def summarize(method, uavs, chargers, rows, horizon):
    starts = {task: [] for task in KIND}
    for row in rows:
        if row["event"] == "task_start":
            starts[row["task"]].append(row["time"])

    # Revisit latency: elapsed time between consecutive services of a
    # location, including mission start to first visit and last visit to the
    # horizon.
    gaps = {"Safety": [], "Quality": []}
    for task, kind in KIND.items():
        previous = 0.0
        for current in starts[task] + [horizon]:
            gaps[kind].append(current - previous)
            previous = current
    every = gaps["Safety"] + gaps["Quality"]

    events = {}
    for row in rows:
        events[row["event"]] = events.get(row["event"], 0) + 1
    ends = [r for r in rows if r["event"] == "mission_end"]
    rounds = [r["rounds"] for r in rows
              if r["event"] == "allocation_converged" and not r.get("forced")]
    sessions = dock_sessions(rows, horizon)
    occupancy = sum(e - s for _, s, e, _ in sessions)
    arrivals = [r["arrival_soc"] for r in rows if r["event"] == "charge_start"]
    minimum_soc = min((r[k] for r in rows
                       for k in ("battery_soc", "arrival_soc", "departure_soc")
                       if k in r), default=1.0)

    # Waiting attributable to charger unavailability.
    wanted, waits = {}, []
    for row in rows:
        agent = row["agent"]
        if row["event"] == "charger_deferred_to_ground":
            wanted[agent] = row["time"]
        elif row["event"] == "charge_start" and agent in wanted:
            waits.append((row["time"] - wanted.pop(agent)) / 60.0)

    completed = {kind: sum(1 for r in rows if r["event"] == "task_complete"
                           and KIND[r["task"]] == kind)
                 for kind in ("Safety", "Quality")}

    return {
        "method": method,
        "uavs": uavs,
        "chargers": chargers,
        "services": events.get("task_complete", 0),
        "safety_services": completed["Safety"],
        "quality_services": completed["Quality"],
        "mean_latency_s": round(sum(every) / len(every), 1),
        "max_latency_s": round(max(every), 1),
        "safety_latency_s": round(sum(gaps["Safety"]) / len(gaps["Safety"]), 1),
        "quality_latency_s": round(
            sum(gaps["Quality"]) / len(gaps["Quality"]), 1),
        # A UAV that required a charging resource and could not obtain one, or
        # that reached one and found it occupied.
        "access_conflicts": (events.get("charger_deferred_to_ground", 0)
                             + events.get("charger_conflict", 0)),
        "wait_for_resource_min": round(sum(waits), 1),
        "charging_events": events.get("charge_start", 0),
        "charger_utilization_pct": round(
            100.0 * occupancy / (chargers * horizon), 1),
        "mean_arrival_soc_pct": round(
            100.0 * sum(arrivals) / len(arrivals), 1) if arrivals else 0.0,
        "minimum_soc_pct": round(100.0 * minimum_soc, 1),
        "depleted_uavs": len({r["agent"] for r in rows
                              if r["event"] == "uav_failed"}),
        "mean_allocation_rounds": round(
            sum(rounds) / len(rounds), 2) if rounds else 0.0,
        "unconverged_auctions": sum(
            1 for r in rows
            if r["event"] == "allocation_converged" and r.get("forced")),
        "messages": sum(r.get("messages", 0) for r in ends),
    }


def main():
    horizon = float(os.environ.get("CR_CBBA_MISSION_DURATION",
                                   MISSION_DURATION_S))
    cells = [(m, u, c) for m in METHODS
             for u in FLEET_SIZES for c in CHARGER_COUNTS]
    data = {cell: load(*cell) for cell in cells}
    present = [cell for cell in cells if data[cell]]
    if not present:
        raise SystemExit("No result logs found under results/")

    problems = []
    for cell in present:
        problems.extend(check(*cell, data[cell], horizon))
    if problems:
        print("CONSISTENCY FAILURES:")
        for item in problems:
            print("  " + item)
        raise SystemExit(1)
    print(f"consistency checks pass for {len(present)} runs "
          f"(horizon {horizon/3600:.2f} h)")

    summaries = [summarize(*cell, data[cell], horizon) for cell in present]
    destination = ROOT / "results" / "ablation_metrics.csv"
    with destination.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=summaries[0].keys())
        writer.writeheader()
        writer.writerows(summaries)
    print("wrote", destination.relative_to(ROOT))
    return summaries


if __name__ == "__main__":
    main()
