#!/usr/bin/env python3
"""Aggregate UAV JSONL logs into paper metrics and plots."""
import csv, json, math, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controllers" / "uav_cbba"))
from mission_config import MISSION_DURATION_S, TASKS as TASK_DEFINITIONS

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
    return {"mode": mode,
            "charger_conflicts": sum(r["event"] == "charger_conflict" for r in rows),
            "charger_waiting_s": round(sum(r.get("duration_s", 0) for r in rows if r["event"] == "charger_wait_end"), 3),
            "revisit_violations": sum(violations.values()),
            "completed_services": sum(map(len, completions.values())),
            "minimum_soc": round(min((r.get("minimum_soc", 1) for r in ends), default=1), 4)}


def plots(data):
    try: import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib unavailable; skipped plots")
        return
    dest = ROOT / "results" / "figures"; dest.mkdir(parents=True, exist_ok=True)
    colors = {"UAV1": "#0072B2", "UAV2": "#E69F00", "UAV3": "#009E73", "UAV4": "#D55E00"}
    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    for ax, mode in zip(axes, MODES):
        starts = {}
        for row in data[mode]:
            if row["event"] == "charge_start": starts[row["agent"]] = row["time"]
            elif row["event"] == "charge_end" and row["agent"] in starts:
                start = starts.pop(row["agent"]); y = int(row["agent"][-1])
                ax.broken_barh([(start, row["time"] - start)], (y - .35, .7), facecolors=colors[row["agent"]])
        ax.set_yticks(range(1, 5), [f"UAV{i}" for i in range(1, 5)])
        ax.set_title(mode.capitalize()); ax.grid(axis="x", alpha=.25)
    axes[-1].set_xlabel("Mission time (s)"); fig.tight_layout()
    fig.savefig(dest / "fig2_charger_timeline.png", dpi=300); plt.close(fig)
    fig, ax = plt.subplots(figsize=(9, 3.5))
    for mode, style in zip(MODES, ("--", "-")):
        starts = task_events(data[mode], "task_start")
        xs = list(range(0, int(MISSION) + 1, 10)); ys = []
        for now in xs:
            ys.append(sum(now - max((t for t in starts[k] if t <= now),
                                    default=0) > TASKS[k][1] for k in TASKS))
        ax.step(xs, ys, where="post", label=mode.capitalize(), linestyle=style)
    ax.set(xlabel="Mission time (s)", ylabel="Overdue tasks", ylim=(-.1, 8.3))
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
