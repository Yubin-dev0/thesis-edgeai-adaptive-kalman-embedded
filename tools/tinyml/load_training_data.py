"""
TinyML training data loader.

Loads E1/E2/E3 run1~3 CSVs (F5-postprocessed), filters out the warm-up
window (seq < 20 where CM-AKF still uses initial fixed R), and assembles
(X_6feat, X_3feat, y) arrays for the 6-feature main model and the
3-feature ablation model.

Thesis references
-----------------
- Table 3-1: F1..F6 feature definitions
- Section 3.5.4: R_label = E[r^2] - P(k|k-1), clamped [1.0, 10000.0]
  → already computed per-step by firmware in CSV column `cm_R`
- Table 3-3: log1p(R) label transform
- Table 4-6: Run 1~3 = training, Run 4~5 = evaluation

This module is the FIRST stage of the training pipeline.  It does NOT
do normalization, train/val split, model build, or quantization — those
belong in `train_ablation.py` (next file).

Usage
-----
    from load_training_data import load_training_set

    data = load_training_set(
        repo_root="C:/Users/shiny/thesis-edgeai-adaptive-kalman-embedded",
        scenarios=("E1",),   # later: ("E1", "E2", "E3")
    )
    X_6 = data["X_6feat"]    # shape (N, 6)
    X_3 = data["X_3feat"]    # shape (N, 3)
    y   = data["y_log1p"]    # shape (N,) — log1p(cm_R)
    meta = data["meta"]      # DataFrame with seq, scenario, run for tracing
"""

from __future__ import annotations
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


# ---------------------------------------------------------------------
# Constants — match thesis Table 3-1 & firmware kalman_filter.h
# ---------------------------------------------------------------------
W_LABEL = 20                # CM-AKF window size; rows with seq<20 use init R
R_MIN, R_MAX = 1.0, 10000.0  # CM-AKF clamp range (matches firmware)

# Feature column mapping (CSV 25-col schema, thesis Table 3-1)
FEATURES_6 = [
    "cm_residual",       # F1 Residual
    "cm_residual_var",   # F2 Residual Variance
    "cm_residual_mean",  # F3 Residual Mean
    "sensor_disagree",   # F4 Sensor Disagreement
    "tof_meas_rate",     # F5 Measurement Rate (use F5-postprocessed CSV!)
    "tof_signal_rate",   # F6 Signal Rate
]
FEATURES_3 = FEATURES_6[:3]  # ablation: residual stats only

LABEL_COL = "cm_R"           # = E[r^2] - P, already clamped, per firmware


# ---------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------
def _scenario_files(repo_root: Path, scenario: str) -> list[Path]:
    """Return Run 1~3 CSV paths for one scenario.

    Currently only E1 has data (and uses F5-postprocessed files).
    E2/E3 will be added once their measurements are done.
    """
    if scenario == "E1":
        proc = repo_root / "experiments" / "E1_baseline" / "processed"
        files = [proc / f"E1_run{n:02d}_f5fixed.csv" for n in (1, 2, 3)]
    elif scenario in ("E2", "E3"):
        # Not yet measured — placeholder path for future
        base = repo_root / "experiments" / f"{scenario}_baseline"
        # E2/E3 firmware will have F5 correct natively, so logs/ directly
        files = [base / "logs" / f"{scenario}_run{n:02d}.csv" for n in (1, 2, 3)]
    else:
        raise ValueError(f"Unknown scenario: {scenario}")

    missing = [f for f in files if not f.exists()]
    if missing:
        raise FileNotFoundError(
            f"{scenario}: missing {len(missing)}/{len(files)} run files.\n"
            + "\n".join(f"  - {f}" for f in missing)
        )
    return files


# ---------------------------------------------------------------------
# Per-run loader
# ---------------------------------------------------------------------
def _load_run(csv_path: Path, scenario: str, run_id: int) -> pd.DataFrame:
    """Load one run CSV, filter warm-up, add metadata columns."""
    df = pd.read_csv(csv_path)

    # --- sanity checks -------------------------------------------------
    required = set(FEATURES_6) | {LABEL_COL, "seq"}
    missing_cols = required - set(df.columns)
    if missing_cols:
        raise ValueError(
            f"{csv_path.name}: missing columns {sorted(missing_cols)}"
        )

    # --- filter warm-up window (seq < W_LABEL) -------------------------
    # During seq=0..19 the CM buffer is filling and cm_R is held at its
    # initial fixed value; including these rows would teach the model
    # to predict the fixed init R instead of the adapted R.
    n_before = len(df)
    df = df[df["seq"] >= W_LABEL].copy()
    n_after = len(df)

    # --- NaN check ----------------------------------------------------
    # Should be none in well-formed CSV but pandas may import empties
    nan_counts = df[FEATURES_6 + [LABEL_COL]].isna().sum()
    if nan_counts.any():
        raise ValueError(
            f"{csv_path.name}: NaN found in feature/label columns:\n"
            f"{nan_counts[nan_counts > 0]}"
        )

    # --- R range check (should be inside CM clamp [1, 10000]) ----------
    r_min, r_max = df[LABEL_COL].min(), df[LABEL_COL].max()
    if r_min < R_MIN - 1e-6 or r_max > R_MAX + 1e-6:
        raise ValueError(
            f"{csv_path.name}: cm_R out of clamp range "
            f"[{r_min:.2f}, {r_max:.2f}] vs expected [{R_MIN}, {R_MAX}]"
        )

    # --- attach metadata ----------------------------------------------
    df["_scenario"] = scenario
    df["_run"] = run_id
    df["_rows_dropped_warmup"] = n_before - n_after  # for reporting

    return df


# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------
def load_training_set(
    repo_root: str | Path,
    scenarios: Iterable[str] = ("E1",),
    verbose: bool = True,
) -> dict:
    """Load and concatenate Run 1~3 of given scenarios.

    Returns
    -------
    dict with keys:
        X_6feat   : (N, 6) np.ndarray  — F1..F6
        X_3feat   : (N, 3) np.ndarray  — F1..F3 (ablation)
        y_log1p   : (N,)   np.ndarray  — log1p(cm_R), the training target
        y_raw     : (N,)   np.ndarray  — raw cm_R (for sanity / inverse check)
        meta      : pd.DataFrame       — seq, _scenario, _run per row
        feature_names_6 : list[str]
        feature_names_3 : list[str]
    """
    repo_root = Path(repo_root)
    if not repo_root.exists():
        raise FileNotFoundError(f"repo_root does not exist: {repo_root}")

    frames: list[pd.DataFrame] = []
    for scenario in scenarios:
        files = _scenario_files(repo_root, scenario)
        for run_id, csv_path in enumerate(files, start=1):
            df = _load_run(csv_path, scenario, run_id)
            frames.append(df)
            if verbose:
                print(
                    f"  {scenario} run{run_id}: "
                    f"{len(df):4d} rows kept "
                    f"({df['_rows_dropped_warmup'].iloc[0]} warm-up dropped) "
                    f"| cm_R range [{df[LABEL_COL].min():.2f}, "
                    f"{df[LABEL_COL].max():.2f}]"
                )

    combined = pd.concat(frames, ignore_index=True)

    # --- assemble arrays ----------------------------------------------
    X_6 = combined[FEATURES_6].to_numpy(dtype=np.float32)
    X_3 = combined[FEATURES_3].to_numpy(dtype=np.float32)
    y_raw = combined[LABEL_COL].to_numpy(dtype=np.float32)
    y_log1p = np.log1p(y_raw).astype(np.float32)

    meta = combined[["seq", "_scenario", "_run"]].rename(
        columns={"_scenario": "scenario", "_run": "run"}
    ).reset_index(drop=True)

    if verbose:
        print()
        print(f"  total: {len(combined)} rows from {len(frames)} runs")
        print(f"  X_6feat shape: {X_6.shape}")
        print(f"  X_3feat shape: {X_3.shape}")
        print(f"  y_log1p shape: {y_log1p.shape}  "
              f"range [{y_log1p.min():.3f}, {y_log1p.max():.3f}]  "
              f"(raw cm_R [{y_raw.min():.2f}, {y_raw.max():.2f}])")
        print()
        print("  per-feature stats (X_6feat):")
        for i, name in enumerate(FEATURES_6):
            col = X_6[:, i]
            print(f"    F{i+1} {name:20s} "
                  f"min={col.min():+10.3f}  max={col.max():+10.3f}  "
                  f"mean={col.mean():+10.3f}  std={col.std():9.3f}")

    return {
        "X_6feat": X_6,
        "X_3feat": X_3,
        "y_log1p": y_log1p,
        "y_raw": y_raw,
        "meta": meta,
        "feature_names_6": FEATURES_6,
        "feature_names_3": FEATURES_3,
    }


# ---------------------------------------------------------------------
# CLI for quick verification
# ---------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    # Default: assume script is run from repo root, or one level deep
    if len(sys.argv) > 1:
        root = Path(sys.argv[1])
    else:
        here = Path(__file__).resolve()
        # Try a few candidates: cwd, parent of script, two levels up
        candidates = [
            Path.cwd(),
            here.parent.parent.parent,   # tools/tinyml/load_training_data.py
            here.parent,
        ]
        root = next((p for p in candidates
                     if (p / "experiments" / "E1_baseline").exists()),
                    candidates[0])

    print(f"Loading training data from: {root}")
    print(f"Scenarios: E1 (E2/E3 pending measurement)")
    print()

    data = load_training_set(root, scenarios=("E1",), verbose=True)

    print()
    print("=" * 60)
    print("Sanity check: F2 vs R_label (must NOT be the same column)")
    print("=" * 60)
    f2 = data["X_6feat"][:, 1]            # cm_residual_var
    r_label_raw = data["y_raw"]            # cm_R
    diff = np.abs(f2 - r_label_raw)
    print(f"  F2 (cm_residual_var) range: [{f2.min():.2f}, {f2.max():.2f}]")
    print(f"  R_label (cm_R)        range: [{r_label_raw.min():.2f}, {r_label_raw.max():.2f}]")
    print(f"  |F2 - R_label|        mean : {diff.mean():.3f}")
    print(f"  |F2 - R_label|        max  : {diff.max():.3f}")
    print(f"  → if mean ≈ 0, trivial-solution risk exists. Should be > 0.")
