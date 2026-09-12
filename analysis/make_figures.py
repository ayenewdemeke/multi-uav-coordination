#!/usr/bin/env python3
"""Render the result figures for the paper from ablation_metrics.csv.

Two-column IEEE format: single-column figures are 3.4 in wide, double-column
7.0 in.  Greyscale-safe markers and line styles throughout.
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results" / "figures"
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.titlesize": 8,
    "legend.fontsize": 7,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "grid.linewidth": 0.4,
    "lines.linewidth": 1.2,
    "lines.markersize": 4,
    "savefig.dpi": 400,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
    "hatch.linewidth": 0.3,
})

FLEETS = (3, 4, 5, 6)
MARKERS = {3: "o", 4: "s", 5: "^", 6: "D"}
STYLE = {"proposed": "-", "battery_only": "--"}
# Palette taken from Figs. 1 and 2: muted blue, green, tan and charcoal.
LINE = {3: "#2E5E8A", 4: "#4E8C6A", 5: "#C4913F", 6: "#444444"}
FILL = {3: "#DCE7F4", 4: "#D0E3D7", 5: "#EEDEC3", 6: "#E1E1E1"}
# Bars are separated by hatch as well as by fill, so the figure survives
# greyscale reproduction.
HATCH = {3: "/////", 4: "\\\\\\\\\\", 5: "xxxxx", 6: "....."}


def load():
    path = ROOT / "results" / "ablation_metrics.csv"
    rows = list(csv.DictReader(path.open()))
    for r in rows:
        for k, v in r.items():
            if k != "method":
                r[k] = float(v) if "." in v else int(v)
    return rows


def series(rows, method, uavs, field):
    sel = sorted((r for r in rows
                  if r["method"] == method and r["uavs"] == uavs),
                 key=lambda r: r["chargers"])
    return [r["chargers"] for r in sel], [r[field] for r in sel]


def panel_labels(fig, axes):
    """Place (a), (b) centred below each panel, as IEEE figures require.

    The label is positioned from the rendered extent of each axes and its
    decorations, so it clears tick labels and axis titles whether or not the
    panel carries them.
    """
    fig.canvas.draw()
    transform = fig.transFigure.inverted()
    for ax, tag in zip(axes, ("(a)", "(b)")):
        extent = ax.get_tightbbox(fig.canvas.get_renderer())
        left, bottom = transform.transform((extent.x0, extent.y0))
        right, _ = transform.transform((extent.x1, extent.y1))
        fig.text((left + right) / 2.0, bottom - 0.018, tag,
                 ha="center", va="top")


def fig_service_and_latency(rows):
    """Throughput and revisit latency of the proposed method."""
    fig, axes = plt.subplots(2, 1, figsize=(3.4, 4.2), sharex=True)
    for uavs in FLEETS:
        x, y = series(rows, "proposed", uavs, "services")
        axes[0].plot(x, y, STYLE["proposed"], color=LINE[uavs],
                     marker=MARKERS[uavs], label=f"{uavs} UAVs")
        x, y = series(rows, "proposed", uavs, "mean_latency_s")
        axes[1].plot(x, [v / 60.0 for v in y], STYLE["proposed"],
                     color=LINE[uavs], marker=MARKERS[uavs],
                     label=f"{uavs} UAVs")
    axes[0].set_ylabel("Completed services")
    axes[1].set_xlabel("Charging resources")
    axes[1].set_ylabel("Mean revisit latency (min)")
    axes[1].set_xticks(list(range(1, 5)))
    axes[0].legend(frameon=False, ncol=2)
    fig.tight_layout(h_pad=2.0)
    panel_labels(fig, axes)
    fig.savefig(OUT / "fig_service_latency.pdf")
    fig.savefig(OUT / "fig_service_latency.png")
    plt.close(fig)


def fig_utilization(rows):
    """Charger utilization against provisioning."""
    fig, ax = plt.subplots(figsize=(3.4, 2.3))
    for uavs in FLEETS:
        x, y = series(rows, "proposed", uavs, "charger_utilization_pct")
        ax.plot(x, y, STYLE["proposed"], color=LINE[uavs],
                marker=MARKERS[uavs], label=f"{uavs} UAVs")
    ax.set_xlabel("Charging resources")
    ax.set_ylabel("Charger utilization (%)")
    ax.set_xticks(list(range(1, 5)))
    ax.set_ylim(0, 105)
    ax.legend(frameon=False, ncol=2)
    fig.tight_layout()
    fig.savefig(OUT / "fig_utilization.pdf")
    fig.savefig(OUT / "fig_utilization.png")
    plt.close(fig)


def fig_conflicts(rows):
    """Charging-resource access conflicts with and without reservation."""
    fig, ax = plt.subplots(figsize=(3.4, 2.3))
    width = 0.1
    offsets = {3: -1.5, 4: -0.5, 5: 0.5, 6: 1.5}
    for uavs in FLEETS:
        x, y = series(rows, "battery_only", uavs, "access_conflicts")
        ax.bar([xi + offsets[uavs] * width for xi in x], y, width,
               color=FILL[uavs], edgecolor="black", linewidth=0.5,
               hatch=HATCH[uavs], label=f"{uavs} UAVs")
    xs, ys = [], []
    for uavs in FLEETS:
        x, y = series(rows, "proposed", uavs, "access_conflicts")
        xs += x
        ys += y
    ax.plot(xs, ys, "x", color="black", markersize=5,
            label="with reservation")
    ax.set_xlabel("Charging resources")
    ax.set_ylabel("Access conflicts")
    ax.set_xticks(list(range(1, 5)))
    ax.set_yticks(list(range(0, 61, 10)))
    ax.set_ylim(0, 78)
    ax.legend(frameon=False, ncol=2, fontsize=6, loc="upper right",
              handlelength=1.6, columnspacing=1.0)
    fig.tight_layout()
    fig.savefig(OUT / "fig_conflicts.pdf")
    fig.savefig(OUT / "fig_conflicts.png")
    plt.close(fig)


def fig_convergence(rows):
    """Allocation rounds and message volume."""
    fig, axes = plt.subplots(2, 1, figsize=(3.4, 4.2), sharex=True)
    for uavs in FLEETS:
        x, y = series(rows, "proposed", uavs, "mean_allocation_rounds")
        axes[0].plot(x, y, STYLE["proposed"], color=LINE[uavs],
                     marker=MARKERS[uavs], label=f"{uavs} UAVs")
        x, y = series(rows, "proposed", uavs, "messages")
        axes[1].plot(x, y, STYLE["proposed"], color=LINE[uavs],
                     marker=MARKERS[uavs], label=f"{uavs} UAVs")
    axes[0].set_ylabel("Mean rounds to convergence")
    axes[1].set_xlabel("Charging resources")
    axes[1].set_ylabel("Coordination messages")
    for ax in axes:
        ax.set_xticks(list(range(1, 5)))
    axes[0].legend(frameon=False, ncol=2)
    fig.tight_layout(h_pad=2.0)
    panel_labels(fig, axes)
    fig.savefig(OUT / "fig_convergence.pdf")
    fig.savefig(OUT / "fig_convergence.png")
    plt.close(fig)


def main():
    rows = load()
    fig_service_and_latency(rows)
    fig_utilization(rows)
    fig_conflicts(rows)
    fig_convergence(rows)
    print("figures written to", OUT.relative_to(ROOT))
    for f in sorted(OUT.glob("*.pdf")):
        print("  ", f.name)


if __name__ == "__main__":
    main()
