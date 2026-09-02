#!/usr/bin/env python3
"""Aggregate UAV JSONL logs into paper metrics and plots."""
import csv, json, math, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "uav_cbba"))
from mission_config import CHARGER_ACCESS_S, MISSION_DURATION_S, TASKS as TASK_DEFINITIONS

TASKS = {task[0]: (task[1], task[5]) for task in TASK_DEFINITIONS}
MODES, MISSION = ("baseline", "proposed"), MISSION_DURATION_S


def load(mode):
    rows = []
    for path in sorted((ROOT / "results" / mode).glob("UAV*.jsonl")):
        rows += [json.loads(line) for line in path.read_text().splitlines() if line]
    return sorted(rows, key=lambda r: (r["time"], r["agent"]))


def task_events(rows, event):
    result = {task: [] for task in TASKS}
    for row in rows:
        if row["event"] == event: result[row["task"]].append(row["time"])
    return result


def summarize(mode, rows):
    starts = task_events(rows, "task_start")
    completions = task_events(rows, "task_complete")
    violations = {"Safety": 0, "Quality": 0}
    for task, (kind, revisit) in TASKS.items():
        previous = 0
        for current in starts[task] + [MISSION]:
            violations[kind] += max(0, math.ceil((current - previous) / revisit) - 1)
            previous = current
    ends = [r for r in rows if r["event"] == "mission_end"]
    cycles = [r["rounds"] for r in rows if r["event"] == "allocation_converged"]
    minimum_soc = min((r.get("minimum_soc", 1) for r in ends), default=1)
    charge_starts = sum(r["event"] == "charge_start" for r in rows)
    lateness = sum(max(0, r["time"] - r["deadline"]) for r in rows
                   if r["event"] == "task_start")
    completed = {
        kind: sum(len(completions[task]) for task, (task_kind, _) in TASKS.items()
                  if task_kind == kind)
        for kind in ("Safety", "Quality")
    }
    return {"mode": mode,
            "charger_conflicts": sum(r["event"] == "charger_conflict" for r in rows),
            "charger_waiting_s": round(sum(r.get("duration_s", 0) for r in rows if r["event"] == "charger_wait_end"), 3),
            "safety_revisit_violations": violations["Safety"],
            "quality_revisit_violations": violations["Quality"],
            "total_revisit_violations": sum(violations.values()),
            "cumulative_start_lateness_s": round(lateness, 3),
            "completed_safety_services": completed["Safety"],
            "completed_quality_services": completed["Quality"],
            "completed_services": sum(completed.values()),
            "reserve_violations": sum(r.get("minimum_soc", 1) < 0.15 for r in ends),
            "minimum_soc_pct": round(100 * minimum_soc, 2),
            "charging_events": charge_starts,
            "charger_utilization_pct": round(
                100 * charge_starts * CHARGER_ACCESS_S / MISSION, 3),
            "charging_reassignments": sum(
                r.get("released_count", 1) for r in rows
                if r["event"] == "charger_conflict_resolved"),
            "reservation_shifts": sum(
                r["event"] == "charger_slot_shifted" for r in rows),
            "mean_allocation_rounds": round(
                sum(cycles) / len(cycles), 3) if cycles else 0,
            "communication_messages": sum(r.get("messages", 0) for r in ends)}


def plots(data):
    try: import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable; skipped plots")
        return
    plt.rcParams.update({"font.family": "Times New Roman", "font.size": 20})
    dest = ROOT / "results" / "figures"; dest.mkdir(parents=True, exist_ok=True)
    colors = {"UAV1": "#0072B2", "UAV2": "#E69F00", "UAV3": "#009E73", "UAV4": "#D55E00"}
    fig, ax = plt.subplots(figsize=(9, 5.25))
    starts = {}
    for row in data["proposed"]:
        if row["event"] == "charge_start": starts[row["agent"]] = row["time"]
        elif row["event"] == "charge_end" and row["agent"] in starts:
            start = starts.pop(row["agent"]); y = int(row["agent"][-1])
            ax.broken_barh([(start / 60.0, (row["time"] - start) / 60.0)],
                           (y - .35, .7), facecolors=colors[row["agent"]])
    ax.set_yticks(range(1, 5), [f"UAV{i}" for i in range(1, 5)])
    ax.set_xlabel("Mission time (min)"); ax.grid(axis="x", alpha=.25); fig.tight_layout()
    fig.savefig(dest / "fig2_charger_timeline.png", dpi=300); plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 5.25))
    for mode, style in zip(MODES, ("--", "-")):
        starts = task_events(data[mode], "task_start")
        xs = list(range(0, int(MISSION) + 1, 10)); ys = []
        for now in xs:
            ys.append(sum(now - max((t for t in starts[k] if t <= now),
                                    default=0) > TASKS[k][1] for k in TASKS))
        ax.step([x / 60.0 for x in xs], ys, where="post",
                label=mode.capitalize(), linestyle=style)
    ax.set(xlabel="Mission time (min)", ylabel="Overdue tasks", ylim=(-.1, 8.3))
    ax.legend(); ax.grid(alpha=.25); fig.tight_layout()
    fig.savefig(dest / "fig3_overdue_tasks.png", dpi=300); plt.close(fig)


def main():
    data = {mode: load(mode) for mode in MODES}
    missing = [m for m in MODES if not data[m]]
    if missing: raise SystemExit("Missing logs for: " + ", ".join(missing))
    rows = [summarize(m, data[m]) for m in MODES]
    destination = ROOT / "results" / "table_ii_metrics.csv"
    with destination.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    plots(data)
    for row in rows: print(json.dumps(row, indent=2))


if __name__ == "__main__": main()
