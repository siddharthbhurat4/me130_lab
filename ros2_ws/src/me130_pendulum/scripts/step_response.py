#!/usr/bin/env python3
"""Free and forced response from the newest steps_*.csv (rod hanging DOWN).

Each step segment holds a constant duty (mode "step", forced) and then releases
the motor to ring down (mode "coast", free). Both are sign-normalized so the
+duty and -duty runs of one magnitude overlay.

Also identifies  theta'' + b*theta' + c*theta = K*u,  P(s) = K/(s^2 + b*s + c).

    ros2 run me130_pendulum step_response.py
"""
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")          # headless: save to file, never open a window
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# full_*.csv is the pre-ROS name, still accepted.
LOG_GLOBS = ["./steps_*.csv", "./build/steps_*.csv",
             "./full_*.csv", "./build/full_*.csv"]
OUT_PNG = "step_response.png"
DPI = 150

SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
SURFACE, INK, INK_2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"

REQUIRED = {"t_s", "mode", "u_cmd", "theta_rad", "theta_dot_rad_s",
            "segment", "step_duty", "freq_hz"}


def newest_log(globs=None, what="step", required=True):
    """Most recent matching log, by the YYYYmmdd_HHMMSS in the filename rather
    than mtime, which a copy does not preserve. None if not required."""
    globs = globs or LOG_GLOBS
    hits = [f for pattern in globs for f in glob.glob(pattern)]
    if not hits:
        if not required:
            return None
        sys.exit("No %s log found in %s\n"
                 "Looked for: %s\n"
                 "Record one first:\n"
                 "  ros2 launch me130_pendulum %s.launch.py deadband:=<from lab 1>"
                 % (what, os.getcwd(), ", ".join(globs),
                    "step_response" if what == "step" else "frequency_response"))
    return max(hits, key=os.path.basename)


def load(path):
    df = pd.read_csv(path)
    missing = REQUIRED - set(df.columns)
    if missing:
        sys.exit("%s is missing column(s): %s" % (path, ", ".join(sorted(missing))))
    # df["mode"], never df.mode -- the latter is a DataFrame method. The numeric
    # codes are older logs, from before the column became text.
    df["mode"] = df["mode"].astype(str).str.strip().replace(
        {"0": "coast", "1": "pd", "2": "step", "3": "sine"})
    return df


def step_segments(df):
    """Yield (duty, forced, free) per step segment: the held portion, then the
    coast portion the firmware inserts before the next step."""
    for _, g in df.groupby("segment", sort=True):
        forced = g[g["mode"] == "step"]
        if forced.empty:
            continue
        duties = forced["step_duty"]
        duty = duties.iloc[int(np.argmax(duties.abs().to_numpy()))]
        if abs(duty) < 1e-9:
            continue
        # Only the coast rows that FOLLOW the drive, not any that precede it.
        free = g[(g["mode"] == "coast") & (g["t_s"] > forced["t_s"].max())]
        yield float(duty), forced, free


def _cumtrap(y, t):
    """Cumulative trapezoid integral of y over t."""
    out = np.zeros_like(y)
    out[1:] = np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))
    return out


def _stiffness_and_gain(df):
    """c (stiffness) and K (gain), by integrating the ODE once rather than
    differentiating it:

        theta'(t) - theta'(t0) = -b*(theta-theta(t0)) - c*Int(theta) + K*Int(u)

    Nothing is differentiated, so noise averages down instead of amplifying.
    Rows are pooled over every segment: driven parts carry K, coast parts pin c.
    """
    use = df[df["mode"].isin(["step", "coast"])]
    if len(use) < 50:
        return None

    t = use["t_s"].to_numpy()
    if len(t) < 3 or not np.all(np.diff(t) > 0):
        return None
    th = use["theta_rad"].to_numpy()
    thd = use["theta_dot_rad_s"].to_numpy()
    u = use["u_cmd"].to_numpy()

    # theta need not be zeroed at the hanging equilibrium, so the plant is
    # really theta'' + b*theta' + c*(theta - theta_eq) = K*u. Integrating leaves
    # c*theta_eq*(t-t0); without the ramp column an unzeroed log collapses to c ~ 0.
    ramp = t - t[0]
    M = np.column_stack([-(th - th[0]), -_cumtrap(th, t), _cumtrap(u, t), ramp])
    coef, *_ = np.linalg.lstsq(M, thd - thd[0], rcond=None)
    b_reg, c, K, k = (float(coef[0]), float(coef[1]), float(coef[2]), float(coef[3]))
    if c <= 0:
        return None
    return c, K, b_reg, k / c


def _damping(df):
    """b from the log decrement of each free ringdown: peaks decay as
    exp(-sigma*t) with sigma = b/2. More accurate than the regression's own b
    column (~4% vs ~16% on the same data). Returns (b, n_ringdowns_used)."""
    sigmas = []
    for _, g in df[df["mode"] == "coast"].groupby("segment"):
        if len(g) < 300:
            continue
        t = g["t_s"].to_numpy()
        y = g["theta_rad"].to_numpy()
        y = y - y.mean()

        peaks = [i for i in range(1, len(y) - 1)
                 if abs(y[i]) > abs(y[i - 1]) and abs(y[i]) >= abs(y[i + 1])
                 and abs(y[i]) > 1e-3]
        if len(peaks) < 3:
            continue
        slope = np.polyfit(t[peaks] - t[peaks[0]], np.log(np.abs(y[peaks])), 1)[0]
        if slope < 0:
            sigmas.append(-slope)

    if not sigmas:
        return None, 0
    # Median, not mean: one mis-detected peak should not move the answer.
    return 2.0 * float(np.median(sigmas)), len(sigmas)


def identify(df):
    """c and K from the integral regression, b from the free response."""
    stiffness = _stiffness_and_gain(df)
    if stiffness is None:
        return None
    c, K, b_reg, theta_eq = stiffness

    b, n_ringdowns = _damping(df)
    if b is None:
        b = b_reg   # no usable ringdown; fall back to the regression column

    wn = np.sqrt(c)
    return dict(K=K, b=b, c=c, b_regression=b_reg, n_ringdowns=n_ringdowns,
                theta_eq=theta_eq, wn_rad=wn, wn_hz=wn / (2 * np.pi),
                zeta=b / (2 * wn))


def report(model):
    if not model:
        print("\nCould not identify a second-order model from this log.")
        return
    print("\nIdentified  theta'' + b*theta' + c*theta = K*u")
    print("            P(s) = K / (s^2 + b*s + c)")
    print("  K = %.4f      (input gain)" % model["K"])
    if abs(model["theta_eq"]) > 0.05:
        print("  note: theta rests %.1f deg off zero -- the IMU was not zeroed at the"
              % np.degrees(model["theta_eq"]))
        print("        hanging position. Handled here, but launch with zero_on_start.")
    if model["n_ringdowns"]:
        print("  b = %.4f      (damping; zeta = %.4f, from %d free ringdowns)"
              % (model["b"], model["zeta"], model["n_ringdowns"]))
    else:
        print("  b = %.4f      (damping; zeta = %.4f, regression fallback)"
              % (model["b"], model["zeta"]))
    print("  c = %.4f      (stiffness; wn = %.3f rad/s = %.3f Hz)"
          % (model["c"], model["wn_rad"], model["wn_hz"]))
    print("\n  Inverted, gravity flips the STIFFNESS term: s^2 + b*s - c")


def headline(fig, model, title, note, x=0.045):
    """Big b / c banner across the top -- the point of the whole plot."""
    fig.text(x, 0.994, title, color=MUTED, fontsize=10, ha="left", va="top")
    if model:
        fig.text(x, 0.967,
                 "K = %.4g     b = %.4g     c = %.4g"
                 % (model["K"], model["b"], model["c"]),
                 color=INK, fontsize=34, fontweight="semibold", ha="left", va="top")
        tail = note
    else:
        fig.text(x, 0.967, "second-order fit unavailable", color=INK,
                 fontsize=28, fontweight="semibold", ha="left", va="top")
        tail = note
    fig.text(x, 0.900, tail, color=MUTED, fontsize=9.5, ha="left", va="top")


def style(ax, xlabel, ylabel, title, subtitle):
    ax.set_facecolor(SURFACE)
    ax.axhline(0, color=GRID, linewidth=1.0, zorder=1)
    ax.set_xlabel(xlabel, color=INK_2, fontsize=9.5)
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9.5)
    ax.set_title(title, color=INK, fontsize=11, loc="left", pad=24)
    ax.text(0.0, 1.015, subtitle, transform=ax.transAxes, color=MUTED,
            fontsize=8.5, ha="left", va="bottom")
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for edge in ("top", "right"):
        ax.spines[edge].set_visible(False)
    for edge in ("left", "bottom"):
        ax.spines[edge].set_color(GRID)


def main():
    path = newest_log()
    print("reading %s" % path)
    df = load(path)

    segs = list(step_segments(df))
    if not segs:
        sys.exit("No step segments in %s.\n"
                 "Was it recorded with step_response.launch.py?" % path)

    # One colour per duty MAGNITUDE: +d and -d are the same experiment mirrored.
    mags = sorted({round(abs(d), 4) for d, _, _ in segs})
    if len(mags) > len(SERIES):
        print("note: %d duty magnitudes but %d palette slots; extras reuse colours."
              % (len(mags), len(SERIES)), file=sys.stderr)
    color_of = {m: SERIES[i % len(SERIES)] for i, m in enumerate(mags)}

    fig = plt.figure(figsize=(12.6, 10.4))
    gs = fig.add_gridspec(2, 2, hspace=0.55, wspace=0.18)
    ax_f = fig.add_subplot(gs[0, 0])
    ax_r = fig.add_subplot(gs[0, 1])
    ax_nf = fig.add_subplot(gs[1, 0])
    ax_nr = fig.add_subplot(gs[1, 1])
    fig.patch.set_facecolor(SURFACE)

    seen, rows = set(), []
    for duty, forced, free in segs:
        mag, sign = round(abs(duty), 4), np.sign(duty)
        color = color_of[mag]
        label = None
        if mag not in seen:
            seen.add(mag)
            label = "|duty| = %.2f" % mag

        tf = forced["t_s"].to_numpy()
        yf_raw = forced["theta_rad"].to_numpy()
        yf = yf_raw * sign
        ax_f.plot(tf - tf[0], np.degrees(yf), color=color, linewidth=1.5,
                  label=label, zorder=3)

        # Shift to where the step started, divide by the duty that drove it. A
        # linear plant collapses every step onto one curve; spread is nonlinearity.
        norm = np.degrees(yf_raw - yf_raw[0]) / duty
        ax_nf.plot(tf - tf[0], norm, color=color, linewidth=1.4,
                   label=label, zorder=3)
        norm_ss = float(np.mean(norm[-min(150, len(norm)):]))

        peak_free = np.nan
        norm_free_ss = np.nan
        if not free.empty:
            tr = free["t_s"].to_numpy()
            yr_raw = free["theta_rad"].to_numpy()
            yr = yr_raw * sign
            ax_r.plot(tr - tr[0], np.degrees(yr), color=color, linewidth=1.5,
                      label=label, zorder=3)
            peak_free = np.degrees(np.abs(yr).max())

            # Same treatment as the forced panel. The release mirrors the step,
            # so these collapse too, settling at minus the forced steady state.
            norm_free = np.degrees(yr_raw - yr_raw[0]) / duty
            ax_nr.plot(tr - tr[0], norm_free, color=color, linewidth=1.4,
                       label=label, zorder=3)
            norm_free_ss = float(np.mean(norm_free[-min(150, len(norm_free)):]))

        rows.append(dict(duty=duty, t_drive=tf[-1] - tf[0],
                         peak_forced=np.degrees(np.abs(yf).max()),
                         peak_free=peak_free, norm_ss=norm_ss,
                         norm_free_ss=norm_free_ss))

    style(ax_f, "Time since step onset (s)", "theta (deg, sign-normalized)",
          "Forced response", "")
    style(ax_r, "Time since release (s)", "theta (deg, sign-normalized)",
          "Free response", "")

    def spread_of(key):
        vals = [r[key] for r in rows if np.isfinite(r[key])]
        if len(vals) < 2:
            return None, "a linear plant collapses these onto one curve"
        lo, hi, mid = min(vals), max(vals), float(np.median(vals))
        pct = 100.0 * (hi - lo) / abs(mid) if mid else 0.0
        return (vals, pct), ""

    ss_info, ss_note = spread_of("norm_ss")
    fr_info, fr_note = spread_of("norm_free_ss")
    ss = ss_info[0] if ss_info else []

    style(ax_nf, "Time since step onset (s)",
          "(theta - theta at onset) / duty   (deg per unit duty)",
          "Normalised forced", ss_note)
    style(ax_nr, "Time since release (s)",
          "(theta - theta at release) / duty   (deg per unit duty)",
          "Normalised free", fr_note)

    for ax in (ax_f, ax_r, ax_nf, ax_nr):
        leg = ax.legend(loc="upper right", frameon=False, fontsize=9)
        for text in leg.get_texts():
            text.set_color(INK_2)

    print("\n%-9s %9s %15s %13s %12s %12s"
          % ("duty", "drive_s", "peak_forced_deg", "peak_free_deg",
             "norm_fwd", "norm_free"))
    print("-" * 78)
    for r in sorted(rows, key=lambda r: r["duty"]):
        print("%-9.2f %9.2f %15.2f %13.2f %12.1f %12.1f"
              % (r["duty"], r["t_drive"], r["peak_forced"], r["peak_free"],
                 r["norm_ss"], r["norm_free_ss"]))
    if fr_info:
        print("\nnormalised free response: %.1f to %.1f deg/duty (%.0f%% spread)."
              % (min(fr_info[0]), max(fr_info[0]), fr_info[1]))
    if len(ss) >= 2:
        print("\nnormalised steady state: %.1f to %.1f deg/duty (%.0f%% spread)."
              % (min(ss), max(ss), 100.0 * (max(ss) - min(ss)) / abs(np.median(ss))))
        if 100.0 * (max(ss) - min(ss)) / abs(np.median(ss)) > 20.0:
            print("  The curves do NOT line up, so the response is not proportional to")
            print("  duty. On a geared rig that is stiction: each step stops where")
            print("  friction holds it, not at the true equilibrium.")

    model = identify(df)
    report(model)

    headline(fig, model, "",
             ""
             "" )
    fig.tight_layout(rect=[0, 0, 1, 0.845])
    fig.savefig(OUT_PNG, dpi=DPI, facecolor=SURFACE, bbox_inches="tight")


if __name__ == "__main__":
    main()
