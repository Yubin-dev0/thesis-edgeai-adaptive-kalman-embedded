#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 7-E : manual-roll integration analysis
--------------------------------------------
Reads the manual-roll CSVs captured with the CLI logger (logger_cli.py)
running the Phase 6 firmware, and reports, per run:

  7-E-1  CSV 18-field integrity     - share of well-formed 18-field rows
  7-E-2  roll speed                 - mean / std over the rolling segment
  7-E-3  Bluetooth drop rate        - estimated from gaps in the seq counter

The roll speed is derived from the encoder counts. The Phase 6 firmware
already corrects the right-encoder sign (the right motor wiring was
reversed), so both channels increase together and the two deltas are
simply ADDED: v = (dL + dR)/2 * MM_PER_PULSE / dt.
NOTE: pre-fix captures (right encoder negative) must NOT be analysed with
this script - the speed would cancel to ~0. Only post-fix runs are valid.

7-E-4 (ADC battery voltage) is measured separately with a multimeter and
is not present in the CSV, so it is not computed here.

How to use
----------
1. Capture one CSV per roll with logger_cli.py (Phase 6 firmware).
2. Save them in the same folder as this script, named:
       7e_run01.csv   7e_run02.csv   ...
3. Run:  python3 analyze_7e.py
   (optional)  python3 analyze_7e.py /path/to/folder

The parser tolerates the firmware's '#' comment lines, the boot banner
(lines not starting with a digit), blank lines and truncated rows. It
also recovers "glued" rows - two CSV records joined without a line break,
which appear as a single 35-field line - by re-splitting them into
18-field records.
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
IDX_SEQ = HEADER.index("seq")
IDX_TS = HEADER.index("timestamp_ms")
IDX_ENCL = HEADER.index("enc_L")
IDX_ENCR = HEADER.index("enc_R")

# Encoder scale: confirmed in Phase 2 (PULSES_PER_REV = 3840).
MM_PER_PULSE = 0.05397

# A row whose instantaneous speed exceeds this (mm/s) is treated as part
# of the rolling segment; slower rows are the start/stop dwell and are
# excluded from the 7-E-2 speed statistics.
ROLL_SPEED_THRESH = 20.0


def _split_records(parts):
    """Split a comma-field list into 18-field records.

    A clean row has exactly 18 fields. A glued row (two records joined
    with no line break) has ~35 fields; re-split it into chunks of 18.
    Returns (list_of_records, leftover_count) where leftover_count counts
    fields that did not form a whole 18-field record (truncated tail).
    """
    records = []
    i = 0
    n = len(parts)
    while n - i >= N_FIELDS:
        records.append(parts[i:i + N_FIELDS])
        i += N_FIELDS
    return records, (n - i)


def parse_csv(path):
    """Return (rows, stats) from one capture file.

    rows  : list of dicts with seq / ts / encL / encR (numeric).
    stats : dict with line counts for the integrity report.

    Skips comment lines ('#'), the header line, the firmware boot banner
    (any line whose first field is not a digit), and blank lines.
    Glued 35-field rows are recovered into two 18-field records.
    """
    rows = []
    n_clean = 0      # lines that were exactly 18 fields
    n_recovered = 0  # extra records recovered from glued lines
    n_skipped = 0    # lines dropped (banner / truncated / non-numeric)

    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")

            # Header line or firmware banner: first field is not a digit.
            if not parts[0].strip().lstrip("-").isdigit():
                continue

            if len(parts) == N_FIELDS:
                recs = [parts]
                n_clean += 1
            elif len(parts) > N_FIELDS:
                recs, _ = _split_records(parts)
                if not recs:
                    n_skipped += 1
                    continue
                n_clean += 1
                n_recovered += len(recs) - 1
            else:
                n_skipped += 1
                continue

            for rec in recs:
                try:
                    rows.append({
                        "seq": int(rec[IDX_SEQ]),
                        "ts": float(rec[IDX_TS]),
                        "encL": float(rec[IDX_ENCL]),
                        "encR": float(rec[IDX_ENCR]),
                    })
                except ValueError:
                    n_skipped += 1

    stats = {
        "n_clean": n_clean,
        "n_recovered": n_recovered,
        "n_skipped": n_skipped,
        "n_rows": len(rows),
    }
    return rows, stats


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


def roll_speed(rows):
    """Per-row instantaneous speed (mm/s) over consecutive valid rows.

    Speed uses ADDED encoder deltas (firmware already sign-corrects the
    right channel). Rows are paired in capture order; a missing seq just
    widens dt, which the timestamp accounts for, so the speed stays valid.
    """
    speeds = []
    for i in range(1, len(rows)):
        a, b = rows[i - 1], rows[i]
        dt = (b["ts"] - a["ts"]) / 1000.0
        if dt <= 0:
            continue
        d_pulse = ((b["encL"] - a["encL"]) + (b["encR"] - a["encR"])) / 2.0
        speeds.append(d_pulse * MM_PER_PULSE / dt)
    return speeds


def seq_drop(rows):
    """Return (received, expected, missing, miss_pct) from the seq counter."""
    seqs = [r["seq"] for r in rows]
    if not seqs:
        return 0, 0, 0, 0.0
    lo, hi = min(seqs), max(seqs)
    expected = hi - lo + 1
    received = len(seqs)
    missing = expected - received
    miss_pct = 100.0 * missing / expected if expected else 0.0
    return received, expected, missing, miss_pct


def analyze(folder):
    print("=" * 70)
    print(" Phase 7-E  -  manual-roll integration analysis")
    print("=" * 70)

    files = sorted(
        f for f in os.listdir(folder)
        if f.startswith("7e_run") and f.endswith(".csv")
    )
    if not files:
        print("  no 7e_run*.csv files found in:", os.path.abspath(folder))
        print("=" * 70)
        return

    per_run = []  # (label, dict-or-None)
    for fname in files:
        path = os.path.join(folder, fname)
        rows, st = parse_csv(path)
        label = fname[:-4]  # strip .csv

        if len(rows) < 2:
            print(f"  [warn] {fname:18s}  not enough data rows")
            per_run.append((label, None))
            continue

        # 7-E-1 integrity: well-formed rows over total parsed lines.
        total_lines = st["n_clean"] + st["n_skipped"]
        integrity = 100.0 * st["n_clean"] / total_lines if total_lines else 0.0

        # 7-E-2 speed over the rolling segment.
        speeds = roll_speed(rows)
        roll = [v for v in speeds if v > ROLL_SPEED_THRESH]
        sp_mean, sp_std = mean_std(roll)
        sp_max = max(roll) if roll else float("nan")

        # 7-E-3 drop rate from seq gaps.
        recv, exp, miss, miss_pct = seq_drop(rows)

        per_run.append((label, {
            "integrity": integrity,
            "n_clean": st["n_clean"],
            "n_skipped": st["n_skipped"],
            "n_recovered": st["n_recovered"],
            "sp_mean": sp_mean,
            "sp_std": sp_std,
            "sp_max": sp_max,
            "recv": recv,
            "exp": exp,
            "miss": miss,
            "miss_pct": miss_pct,
        }))

        note = ""
        if st["n_recovered"]:
            note = f"({st['n_recovered']} rows recovered from glued lines)"
        print(f"  [ ok ] {fname:18s}  {st['n_clean']} rows {note}")

    print("-" * 70)
    print(" Per-run results  (paste into the Phase 7 record table)")
    print("-" * 70)
    print(f" {'Run':10s} {'integrity':>10s} {'speed mean+/-std':>20s} "
          f"{'max':>9s} {'drop':>9s}")
    print(f" {'':10s} {'(>=99%)':>10s} {'(mm/s)':>20s} "
          f"{'(mm/s)':>9s} {'(<=1%)':>9s}")
    print("-" * 70)
    for label, d in per_run:
        if d is None:
            print(f" {label:10s} {'-- no data --':>10s}")
            continue
        integ = f"{d['integrity']:.2f}%"
        spd = f"{d['sp_mean']:.1f} +/- {d['sp_std']:.1f}"
        smax = f"{d['sp_max']:.1f}"
        drop = f"{d['miss_pct']:.1f}%"
        print(f" {label:10s} {integ:>10s} {spd:>20s} {smax:>9s} {drop:>9s}")
    print("-" * 70)

    # Across-run speed consistency (7-E-2 asks for the 5-roll spread).
    run_means = [d["sp_mean"] for _, d in per_run
                 if d is not None and not math.isnan(d["sp_mean"])]
    if len(run_means) >= 2:
        m, s = mean_std(run_means)
        print(f" Across runs : mean {m:.1f} mm/s, std +/- {s:.1f} mm/s "
              f"({len(run_means)} runs)")
        print("-" * 70)

    print(" Notes:")
    print("  - 7-E-1 integrity : well-formed 18-field rows / parsed lines.")
    print("  - 7-E-2 speed     : rolling segment only (>20 mm/s); start/stop")
    print("                      dwell excluded. Recorded, not gated.")
    print("  - 7-E-3 drop      : whole-frame loss from seq gaps. The link is")
    print("                      clean when stationary; loss appears while")
    print("                      the wheels turn (motor rotation noise).")
    print("  - 7-E-4 ADC voltage is a separate multimeter reading.")
    print("=" * 70)


if __name__ == "__main__":
    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    analyze(folder)
