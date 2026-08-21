#!/usr/bin/env python3
"""Aggregate UAV JSONL logs into paper metrics and plots."""
import csv, json, math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TASKS = {"S1": ("Safety", 120), "S2": ("Safety", 150),
         "S3": ("Safety", 150), "S4": ("Safety", 200),
         "Q1": ("Quality", 400), "Q2": ("Quality", 400),
         "Q3": ("Quality", 500), "Q4": ("Quality", 500)}
MODES, MISSION = ("baseline", "proposed"), 3600


def load(mode):
    rows = []
    for path in sorted((ROOT / "results" / mode).glob("UAV*.jsonl")):
        rows += [json.loads(line) for line in path.read_text().splitlines() if line]
    return sorted(rows, key=lambda r: (r["time"], r["agent"]))


def completions(rows):
    result = {task: [] for task in TASKS}
    for row in rows:
        if row["event"] == "task_complete": result[row["task"]].append(row["time"])
    return result


def summarize(mode, rows):
    done = completions(rows); violations = {"Safety": 0, "Quality": 0}
    for task, (kind, revisit) in TASKS.items():
        previous = 0
        for current in done[task] + [MISSION]:
            # Count each prescribed revisit deadline missed within the gap,
            # rather than merely classifying the entire gap as late once.
            violations[kind] += max(0, math.ceil((current - previous) / revisit) - 1)
            previous = current
    cycles = [r["convergence_s"] for r in rows if r["event"] == "allocation_cycle"]
    ends = [r for r in rows if r["event"] == "mission_end"]
    return {"mode": mode,
            "charger_conflicts": sum(r["event"] == "charger_conflict" for r in rows),
            "charger_waiting_s": round(sum(r.get("duration_s", 0) for r in rows if r["event"] == "charger_wait_end"), 3),
            "safety_revisit_violations": violations["Safety"],
            "quality_revisit_violations": violations["Quality"],
            "completed_services": sum(map(len, done.values())),
            "charging_reassignments": sum(r["event"] == "charger_conflict_resolved" for r in rows),
            "mean_local_allocation_compute_s": round(sum(cycles) / len(cycles), 6) if cycles else 0,
            "consensus_window_s": 8.0,
            "communication_messages": sum(r.get("messages", 0) for r in ends),
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
        done = completions(data[mode]); xs = list(range(0, MISSION + 1, 10)); ys = []
        for now in xs:
            ys.append(sum(now - max((t for t in done[k] if t <= now), default=0) > TASKS[k][1] for k in TASKS))
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
