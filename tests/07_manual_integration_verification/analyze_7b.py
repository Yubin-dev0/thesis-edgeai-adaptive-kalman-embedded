#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 7-B : wall material pre-measurement analysis + chart
----------------------------------------------------------
Reads the 4 static-measurement CSVs captured with the Phase 6 firmware
(white / black / gray woodlock + transparent acrylic) and reports, for
each material, the VL53L0X signal-rate mean and standard deviation.
For the acrylic sample it also reports the share of non-zero range_status
rows, since the transparent surface is expected to fail ranging often.

In addition to the text table, the script saves a bar chart
(7b_signal_rate.png) comparing the signal rate of the four materials,
with the standard deviation drawn as an error bar. The chart is suitable
for use as a thesis figure.

How to use
----------
1. Capture one 5-second CSV per material with the Phase 6 firmware.
2. Save them in the same folder as this script, named:
       7b_white.csv     7b_black.csv     7b_gray.csv     7b_acrylic.csv
3. Run:  python3 analyze_7b.py
   (optional)  python3 analyze_7b.py /path/to/folder

The script tolerates the firmware's '#' comment lines, blank lines and
truncated rows, so the raw PuTTY / serial_logger capture can be fed in
directly without manual cleaning. Materials whose CSV is missing are
simply skipped, so the chart can be produced before acrylic is remeasured.
"""

import sys
import os
import math

# CSV column layout produced by the Phase 6 firmware (CSV_HEADER, 18 fields)
HEADER = [
    "seq", "timestamp_ms", "enc_L", "enc_R", "pos_L_mm", "pos_R_mm",
    "tof_dist_mm", "tof_status", "tof_signal_mcps", "tof_ambient_mcps",
    "kf_estimate", "kf_covariance", "residual", "residual_var",
    "residual_mean", "kalman_gain", "innovation_cov", "scenario_id",
]
N_FIELDS = len(HEADER)
IDX_STATUS = HEADER.index("tof_status")
IDX_SIGNAL = HEADER.index("tof_signal_mcps")

# Files expected, in report order. Label is what gets printed and plotted.
MATERIALS = [
    ("7b_white.csv",   "White"),
    ("7b_black.csv",   "Black"),
    ("7b_gray.csv",    "Gray"),
    ("7b_acrylic.csv", "Acrylic"),
]

CHART_NAME = "7b_signal_rate.png"


def parse_csv(path):
    """Return (signal_values, status_values, skipped) from one capture file.

    Skips comment lines ('#'), the header line, blank lines and any row
    that does not have exactly N_FIELDS numeric-ish columns.
    """
    signals = []
    statuses = []
    skipped = 0
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) != N_FIELDS:
                skipped += 1
                continue
            try:
                status = int(float(parts[IDX_STATUS]))
                signal = float(parts[IDX_SIGNAL])
            except ValueError:
                skipped += 1
                continue
            statuses.append(status)
            signals.append(signal)
    return signals, statuses, skipped


def mean_std(values):
    """Sample mean and standard deviation (ddof=1). Pure Python, no deps."""
    n = len(values)
    if n == 0:
        return float("nan"), float("nan")
    m = sum(values) / n
    if n == 1:
        return m, 0.0
    var = sum((v - m) ** 2 for v in values) / (n - 1)
    return m, math.sqrt(var)


def make_chart(rows, folder):
    """Save a bar chart of signal-rate mean +/- std for available materials.

    'rows' is the list of (label, data-or-None) produced by analyze().
    Returns the saved path, or None if matplotlib is unavailable or there
    is nothing to plot.
    """
    have = [(label, d) for label, d in rows if d is not None]
    if not have:
        print("  [chart] no data to plot - chart skipped")
        return None

    try:
        import matplotlib
        matplotlib.use("Agg")  # no display needed, just save to file
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [chart] matplotlib not installed - chart skipped")
        print("          install with:  pip install matplotlib")
        return None

    labels = [label for label, _ in have]
    means = [d["sig_mean"] for _, d in have]
    stds = [d["sig_std"] for _, d in have]

    # Bar colors loosely echo each material so the figure reads at a glance.
    color_map = {
        "White":   "#cfcfcf",
        "Black":   "#4a4a4a",
        "Gray":    "#8c8c8c",
        "Acrylic": "#7fb6d4",
    }
    colors = [color_map.get(label, "#5b8def") for label in labels]

    fig, ax = plt.subplots(figsize=(6.0, 4.0))
    xpos = list(range(len(labels)))
    ax.bar(
        xpos, means, yerr=stds, capsize=6,
        color=colors, edgecolor="#333333", linewidth=0.8,
        error_kw={"ecolor": "#333333", "elinewidth": 1.0},
    )

    top = max(m + s for m, s in zip(means, stds))

    # Numeric label above each bar: mean +/- std
    for x, m, s in zip(xpos, means, stds):
        ax.text(x, m + s + top * 0.03,
                f"{m:.3f}\n+/-{s:.3f}",
                ha="center", va="bottom", fontsize=9)

    ax.set_xticks(xpos)
    ax.set_xticklabels(labels)
    ax.set_ylabel("VL53L0X signal rate (MCPS)")
    ax.set_title("Phase 7-B  wall material signal rate at 500 mm")
    ax.set_ylim(0, top * 1.25)
    ax.grid(axis="y", linestyle=":", linewidth=0.6, alpha=0.7)
    ax.set_axisbelow(True)

    fig.tight_layout()
    out_path = os.path.join(folder, CHART_NAME)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def analyze(folder):
    print("=" * 64)
    print(" Phase 7-B  -  wall material pre-measurement (500 mm, static)")
    print("=" * 64)

    rows = []
    for fname, label in MATERIALS:
        path = os.path.join(folder, fname)
        if not os.path.isfile(path):
            print(f"  [skip] {fname:16s}  file not found")
            rows.append((label, None))
            continue

        signals, statuses, skipped = parse_csv(path)
        n = len(signals)
        if n == 0:
            print(f"  [warn] {fname:16s}  no valid data rows")
            rows.append((label, None))
            continue

        sig_mean, sig_std = mean_std(signals)
        nonzero = sum(1 for s in statuses if s != 0)
        nonzero_pct = 100.0 * nonzero / n

        rows.append((label, {
            "n": n,
            "skipped": skipped,
            "sig_mean": sig_mean,
            "sig_std": sig_std,
            "nonzero_pct": nonzero_pct,
        }))

        note = f"({skipped} bad rows skipped)" if skipped else ""
        print(f"  [ ok ] {fname:16s}  {n} valid rows {note}")

    print("-" * 64)
    print(" Result table  (paste into the Phase 7 record table)")
    print("-" * 64)
    print(f" {'Material':22s} {'signal_rate (MCPS)':24s} {'status!=0':>10s}")
    print(f" {'':22s} {'mean +/- std':24s} {'':>10s}")
    print("-" * 64)
    for label, data in rows:
        if data is None:
            print(f" {label:22s} {'-- no data --':24s} {'--':>10s}")
            continue
        sig = f"{data['sig_mean']:.3f} +/- {data['sig_std']:.3f}"
        pct = f"{data['nonzero_pct']:.1f}%"
        print(f" {label:22s} {sig:24s} {pct:>10s}")
    print("-" * 64)
    print(" Notes:")
    print("  - signal_rate mean/std is the 7-B-1..4 primary record.")
    print("  - status!=0 share is the 7-B-4 acrylic record; for the three")
    print("    woodlock samples it should be near 0% (sanity check).")
    print("-" * 64)

    chart_path = make_chart(rows, folder)
    if chart_path:
        print(f"  [chart] saved: {chart_path}")
    print("=" * 64)


if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    analyze(folder)
