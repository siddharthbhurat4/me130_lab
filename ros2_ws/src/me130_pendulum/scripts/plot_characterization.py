#!/usr/bin/env python3
"""Plot the motor deadband from the motor_characterization down-sweep.

Left panel is the measured duty vs output speed; right panel is the same data
plotted against the controller command u, i.e. with the deadband compensated
out (the inverse of applyDeadband() in balance_encoder_imu.cpp).

Usage:
    python3 plot_characterization.py [--csv FILE] [--out FILE] [--deadband D]
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")          # headless: save to file, never open a window
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Must match MOVE_THRESHOLD_CPS in motor_characterization.cpp.
MOVE_THRESHOLD_CPS = 8.0

C_DOWN = "#eb6834"
SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"

LABEL = "motor characterization sweep"


def load(path):
    if not os.path.exists(path):
        sys.exit("No such file: %s\nRun motor_characterization first." % path)
    df = pd.read_csv(path)

    required = {"phase", "direction", "duty", "counts_per_second", "output_rpm"}
    missing = required - set(df.columns)
    if missing:
        sys.exit("CSV is missing column(s): %s" % ", ".join(sorted(missing)))

    # CSVs from the older pwmpwm build predate the 'run' column.
    if "run" not in df.columns:
        df["run"] = 1
    if df.empty:
        sys.exit("CSV has a header but no data rows.")

    df = df[df.phase == "DOWN"].copy()
    if df.empty:
        sys.exit("CSV has no DOWN-phase rows.")

    # Signed command. The +0.0 folds -0.0 (from direction -1 at duty 0) onto
    # 0.0 so the two directions share a single origin sample.
    df["cmd"] = df.direction * df.duty + 0.0
    return df


def stop_duty(sub):
    """Duty magnitude where the shaft stops, averaged over runs.

    Mirrors the C++ logic: scanning down, the first duty whose |cps| falls
    below the move threshold. Runs that never stop are left out of the mean.
    """
    stops = []
    for _, run in sub.groupby("run"):
        down = run.sort_values("duty", ascending=False)
        stopped = down[down.counts_per_second.abs() < MOVE_THRESHOLD_CPS]
        if not stopped.empty:
            stops.append(stopped.duty.iloc[0])
    return float(np.mean(stops)) if stops else None


def deadband(df):
    """Single symmetric deadband magnitude: the two directions' stop duties, averaged.

    The forward and reverse stop duties differ a little, but applyDeadband()
    applies one D to both, so the plot shows one symmetric band too.
    """
    found = [stop_duty(df[df.direction > 0]), stop_duty(df[df.direction < 0])]
    found = [abs(v) for v in found if v is not None]
    return float(np.mean(found)) if found else None


def uncompensate(cmd, D):
    """Map measured duty back to the controller command u that would produce it.

    Inverse of applyDeadband(). Everything inside the deadband maps to u=0,
    which is what collapses the flat middle.
    """
    if D is None or D <= 0.0 or D >= 1.0:
        return cmd
    return np.sign(cmd) * np.maximum(np.abs(cmd) - D, 0.0) / (1.0 - D)


def draw(ax, df, D=None):
    """Mean curve across runs, with a min-max band showing run-to-run spread."""
    g = df.groupby("cmd").output_rpm.agg(["mean", "min", "max"]).sort_index()
    x = uncompensate(g.index.to_numpy(), D)

    if (g["max"] - g["min"]).max() > 0:
        ax.fill_between(x, g["min"], g["max"], color=C_DOWN, alpha=0.15,
                        linewidth=0, zorder=2)
    ax.plot(x, g["mean"], color=C_DOWN, linewidth=1.8, marker="o",
            markersize=4.5, markeredgecolor=SURFACE, markeredgewidth=0.8,
            label=LABEL, zorder=3)


def style(ax, title, xlabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=11, loc="left", pad=12)
    ax.axhline(0, color=GRID, linewidth=1.0, zorder=1)
    ax.axvline(0, color=GRID, linewidth=1.0, zorder=1)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9.5)
    ax.set_xlim(-1.04, 1.04)
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for edge in ("top", "right"):
        ax.spines[edge].set_visible(False)
    for edge in ("left", "bottom"):
        ax.spines[edge].set_color(GRID)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="motor_characterization.csv")
    ap.add_argument("--out", default="motor_characterization.png")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--deadband", type=float, default=None,
                    help="D used for the compensated panel; default is the mean "
                         "measured stop magnitude")
    args = ap.parse_args()

    df = load(args.csv)
    D = args.deadband if args.deadband is not None else deadband(df)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12.6, 6.0), sharey=True)
    fig.patch.set_facecolor(SURFACE)

    # ---- headline: the deadband value the whole plot is about ----
    fig.text(0.5, 0.975, "n/a" if D is None else "Deadband = \u00b1%.3f" % D, ha="center",
             va="top", color=INK, fontsize=34, fontweight="semibold")

    # ---- left: measured ----
    style(ax1, "Measured", "Commanded duty  (signed: sign = direction)")
    if D is not None:
        ax1.axvspan(-D, D, color=MUTED, alpha=0.10, linewidth=0, zorder=0)
        ax1.text(0.0, 0.965, "deadband", transform=ax1.get_xaxis_transform(),
                 color=INK_2, fontsize=9, ha="center", va="top")
    draw(ax1, df)
    if D is not None:
        for value in (-D, D):
            ax1.axvline(value, color=C_DOWN, linestyle="--", linewidth=1.1,
                        alpha=0.55, zorder=2)
    ax1.set_ylabel("Output shaft speed (RPM, signed)", color=INK_2, fontsize=9.5)
    span = float(df.output_rpm.abs().max()) * 1.06
    ax1.set_ylim(-span, span)

    # ---- right: deadband compensated out ----
    style(ax2, "Deadband removed", "Controller command u  (deadband compensated)")
    draw(ax2, df, D=D)

    handles, labels = ax1.get_legend_handles_labels()
    if handles:
        leg = fig.legend(handles, labels, loc="lower center", ncol=len(handles),
                         frameon=False, fontsize=9.5, bbox_to_anchor=(0.5, 0.055))
        for text in leg.get_texts():
            text.set_color(INK_2)

    fig.tight_layout(rect=[0, 0.10, 1, 0.865])
    fig.savefig(args.out, dpi=args.dpi, facecolor=SURFACE, bbox_inches="tight")
    print("wrote %s  (%d rows, D = %s)"
          % (args.out, len(df), "n/a" if D is None else "%.3f" % D))


if __name__ == "__main__":
    main()
