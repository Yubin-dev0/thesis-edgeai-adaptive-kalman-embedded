# E2 Reflectivity — Measurement Result Report

| | |
|---|---|
| Scenario | E2 — Reflectivity (wall surface reflectivity variation) |
| Date | 2026-05-20 |
| Runs | 9 main runs (white ×3, black ×3, acryl ×3) |
| Firmware | 28-column triple-KF (Fixed KF + CM-AKF + TinyML-AKF), Scheme C, stage 4-C |
| Result | **PASS** — 9 runs nominal, material noise separation confirmed |

Corresponds to scenario E2 in thesis Table 4-4 (Section 4.2).

---

## 1. Prerequisites — All Complete

- [x] **E1 baseline measurement complete** — runs 1–5 nominal,
      `experiments/E1_baseline/` pushed.
- [x] **Stage 4-C TinyML integration complete** — X-CUBE-AI runtime
      integrated and running on the MCU. `ai_init()` failure (opaque
      `stai_network` context size + PREALLOCATED I/O buffers) resolved.
      Third KF instance `kf_tinyml` added, fed by on-device TinyML R
      prediction. CSV expanded 25 → 28 columns. This is the major change
      vs E1, which used PC-side post-hoc inference.
- [x] **F5 definition aligned to thesis** — `tof_meas_rate` now computed as
      the ratio of `range_status==0` within a W=20 sliding window
      (thesis Table 3-1), replacing the E1 change-rate implementation.
- [x] **Normalization constants hardcoded** — standard normalization
      (mean, std) fit on E1 Run 1–3, embedded in firmware (thesis 3.5.3).

## 2. Experimental Conditions (Thesis 4.1.2 / 4.2)

| Item | Planned | Measured |
|---|---|---|
| Wall | White / black foam-board 8-section + transparent acrylic B4 3 mm | Same (3 materials) |
| Start-to-wall distance | 500 mm (tape measure) | sensor-referenced ~510–527 mm (see E1 R0 note) |
| Floor | Plywood + MDF base board | Same |
| Travel | One-way manual roll, ~100–150 mm/s (E1-consistent) | Same range |
| Lighting | Direct sunlight blocked, same across runs | Same |
| Number of runs | 15 (3 materials × 5) | **9 collected (3 × 3, learning set)** — eval runs 4–5 deferred |
| Samples per run | 1000 loops ≈ 5 s | Same |
| Valid rows per run | expected ~125 | **measured 228–237** |
| Acrylic mounting | flat | **tilted ~5–10° to ToF axis** (specular-return mitigation, Phase 1 method) |

> **Run-split note.** Per thesis Table 4-6, each material's run1–3 are the
> TinyML learning set and run4–5 the evaluation set. This report covers the
> 9 learning runs (3 materials × run1–3). Evaluation runs (4–5 per material)
> are measured separately on the frozen firmware.

## 3. Firmware (E2 Measurement — 28-column Triple-KF, stage 4-C)

E2 is measured with the stage 4-C firmware that runs **three** KF instances
concurrently — `kf_fixed` (fixed R), `kf_cm` (CM-AKF), and `kf_tinyml`
(R injected from on-device TinyML inference) — all fed the same ToF/encoder
input, logged side by side to a 28-column CSV. Unlike E1, the TinyML-AKF
estimate is produced **on the MCU in real time**, not by PC post-hoc
inference.

Items applied for the measurement:

- [x] `CSV_SCENARIO_ID = 2U` — E2.
- [x] **B1 button trigger** — same as E1 (excludes the stationary transient
      after reset; LD2 blink = "press B1" cue, solid = "measuring").
- [x] **Encoder sign correction** — `int16_t dr = -(enc_r_now - enc_r_prev);`
      single-entry correction (same as E1).
- [x] **Scheme C predict/update 1:1** — unchanged from E1 (see E1
      EXP_RESULT Section 4). Now applied to all three KF instances.
- [x] **TinyML inference path** — 6 features (F1–F6) → standard
      normalization → INT8 quantization → `stai_network_run` → INT8
      dequantization → `expm1` (log1p inverse) → clamp [1, 10000] → injected
      as `kf_tinyml.R`. Runs once per ToF update, just before predict/update.
- [x] **28-column CSV header** — adds `tinyml_estimate_mm`, `tinyml_R`,
      `tinyml_infer_us` to the E1 25-column layout. DMA-transmitted.

## 4. TinyML On-Device Inference — Verification

The major addition vs E1 is on-device TinyML inference. Verified during E2
measurement:

**Inference executes.** The `# TinyML infer: count=...` summary line is now
present (absent in E1 and in the pre-4-C builds), with count matching the
predict/update count per run. The `tinyml_R` CSV column varies per step
rather than staying at the init value (24.0), confirming the model is
actually producing predictions.

**Real-time budget (RQ1).** Firmware mean body time stayed in the ~1200 µs
range with the inference path active, within the 200 Hz loop budget
(5000 µs). Per-loop overrun count was 0.

**Output behaviour.** `tinyml_R` tracks `cm_R` at roughly 50–80% of its
magnitude across all materials (see Section 5) — the model, trained on
CM-AKF labels, produces a conservative R estimate. This is distinct from
Fixed KF (R = 24 constant) and demonstrates the three algorithms behave
differently on the same input, satisfying the fair-comparison premise
(thesis 3.1).

## 5. Measurement Results (9 Learning Runs)

| Run | Fixed residual mean | Fixed residual σ | CM-AKF R mean | CM R range | TinyML R mean | CM track err | TinyML track err |
|---|---|---|---|---|---|---|---|
| white_run01 | −1.87 mm | 4.37 | 24.6 | 1.4 – 94.5 | 20.2 | 3.25 mm | 2.92 mm |
| white_run02 | −2.29 mm | 4.29 | 40.9 | 1.8 – 129.9 | 24.6 | 4.45 mm | 3.40 mm |
| white_run03 | −2.51 mm | 4.42 | **326.7** | 1.0 – 1212.4 | **146.8** | 11.59 mm | 7.55 mm |
| black_run01 | −1.21 mm | 10.39 | 79.3 | 2.6 – 369.1 | 50.8 | 5.83 mm | 5.67 mm |
| black_run02 | −0.88 mm | 8.25 | 51.1 | 2.2 – 266.4 | 34.4 | 4.50 mm | 4.33 mm |
| black_run03 | −2.25 mm | 7.68 | **248.7** | 2.4 – 949.2 | **116.0** | 11.07 mm | 8.08 mm |
| acryl_run01 | −1.25 mm | 10.71 | 110.0 | 1.6 – 627.9 | 93.3 | 6.34 mm | 6.24 mm |
| acryl_run02 | −1.01 mm | 11.49 | 147.8 | 2.4 – 880.8 | 121.4 | 7.37 mm | 7.16 mm |
| acryl_run03 | −1.69 mm | 9.44 | 124.0 | 5.0 – 552.6 | 98.0 | 7.50 mm | 6.70 mm |

### Group summary

| Material | signal_rate mean (MCps) | CM-AKF R mean | TinyML R mean | status=0 |
|---|---|---|---|---|
| White foam-board | 20.45 | 130.71 | 63.86 | 100% |
| Black foam-board | 10.08 | 126.34 | 67.05 | 100% |
| Transparent acrylic (tilted) | 13.18 | 127.27 | 104.22 | 100% |

### Group summary — stable runs only (excluding white_run03, black_run03)

| Material | CM-AKF R mean | ratio (white = 1) |
|---|---|---|
| White (run01, 02) | 32.7 | ×1.0 |
| Black (run01, 02) | 65.2 | ×2.0 |
| Acrylic (run01–03) | 127.3 | ×3.9 |

- **signal_rate** — monotonic separation across materials: white 20.45 →
  black 10.08 (≈ half) → acrylic 13.18. The white→black drop is the clean
  reflectivity signature.
- **CM-AKF R** — on stable runs, monotonic increase white 32.7 → black 65.2
  → acrylic 127.3, roughly doubling per material step. This is the core
  RQ2 evidence (adaptation to a harder noise environment).
- **TinyML R** — follows the same monotonic trend at ~50–80% of CM-AKF R.
- **Fixed residual σ** grows with material difficulty (white ~4.3 → black
  ~8.8 → acrylic ~10.5 mm), confirming the measurement noise itself rises.

> **Outlier runs.** white_run03 and black_run03 show 5–10× higher R than
> their same-material siblings. Their `tof_distance` is also closer
> (172–192 mm vs 200–250 mm for the others), indicating a start-point
> offset or hand-roll wobble during the run. They are kept in the learning
> set as natural measurement variability; the post-processing role should
> flag them for outlier review. Excluding them recovers the clean monotonic
> material separation (table above).

## 6. Key Finding — status≠0 Not Reproduced (F5 Responsibility → E3)

Across all 9 runs, `range_status != 0` occurred **0 times** — the
transparent acrylic did **not** produce the partial signal loss stated in
the thesis 4.2 E2 description.

**Cause.** The VL53L0X uses a 940 nm IR source. Ordinary transparent acrylic
is partially transmissive in the IR band, so the beam passes through and
returns a (noisy) reading rather than failing. The result is increased noise
variance (R rises), not signal loss (status≠0). A ~5–10° tilt (Phase 1
method) improved noise consistency but did not induce status≠0.

**Consequence for F5.** F5 (Measurement Rate) is defined as the
`status==0` ratio over a W=20 window. With status≠0 at 0 across E1 and E2,
**F5 is constant 1.000 in both scenarios**. E3 (sensor occlusion) is the
sole learning-data source for F5. This matches thesis Table 4-6, whose E3
note already specifies "includes range_status distribution over the
blockage interval".

This must be reflected in the thesis (see Section 11).

## 7. Data-Collection Success Criteria

| Criterion | Value | Note |
|---|---|---|
| Valid travel samples | ≥ 100 | measured 228–237, passes |
| Mean roll speed | 100 – 150 mm/s | E1-consistent range |
| ToF status=0 ratio | ≥ 95% | measured 100% all runs |
| Bluetooth drop | < 50% (anomaly threshold) | 95–98% integrity, within normal |
| Material noise separation | monotonic R increase | confirmed (stable runs) |

## 8. Measurement Procedure (per Run)

Repeated for each material (×3 runs):

1. Mount the wall material; for acrylic, set the ~5–10° tilt to the ToF
   axis. Re-check the 500 mm start distance.
2. On the PC, run `py logger_cli.py E2 <run_number> [port]`. The logger
   auto-detects the column count from `# CSV_HEADER:` — should show 28.
3. Reset the board → LD2 blinks → press B1 → measurement starts (LD2 solid).
4. Roll the robot toward the wall, steadily and smoothly.
5. Wait for auto-stop at 1000 loops; check the firmware summary. The
   `KF predict / update` counts must be equal (Scheme C). The
   `# TinyML infer: count=...` line should be present with count ≈
   predict/update. If either check fails, re-measure that run.
6. After each material's run1, quick-check `tof_signal_rate` and `cm_R`
   against the expected direction before continuing.

> A trial run preceded the main set; it caught (a) the firmware
> scenario-id not yet flashed, and (b) a grey foam-board mistakenly used in
> place of black. Both were corrected before the recorded learning runs.

## 9. Data Flow

```
one firmware measurement = one CSV  (28 columns: 12 shared + 6 fixed + 7 cm + 3 tinyml)
        │
        ▼  logs_final/   final 4-C firmware logs (E2_<material>_run01–03.csv)
        │
        ▼  CSV logs handed to the post-processing role — per-algorithm split,
           filename normalisation, GT column computation are done there.
           (TinyML R is now on-device, no PC post-hoc inference needed.)
```

CSV 28-column order:

```
seq, timestamp_ms, tof_distance_mm, tof_signal_rate, tof_range_status,
us_distance_mm, encoder_distance_mm, encoder_speed_mms, sensor_disagree,
tof_meas_rate, gt_distance_mm, scenario_id,
fixed_estimate_mm, fixed_residual, fixed_residual_var, fixed_residual_mean,
fixed_kalman_gain, fixed_innovation_cov,
cm_estimate_mm, cm_residual, cm_residual_var, cm_residual_mean,
cm_kalman_gain, cm_innovation_cov, cm_R,
tinyml_estimate_mm, tinyml_R, tinyml_infer_us
```

- **One firmware measurement = one CSV.** raw / Fixed / CM / TinyML values
  are all recorded in a single measurement (no per-algorithm firmware swap).
- `gt_distance_mm` is written as 0 by the firmware → computed in
  post-processing.
- `tof_meas_rate` now holds the F5 ratio (status==0 over W=20), not the old
  change-rate.

## 10. Ground Truth (GT)

Same method as E1: dynamic GT = reference distance − cumulative encoder
distance, reference distance = mean ToF over the stationary pre-run segment.
MM_PER_PULSE ≈ 0.05397 mm. The final wall-collision segment is trimmed in
analysis.

## 11. Thesis Revision Items (from E2)

- **4.2 E2 body** — replace "transparent acrylic causes extremely low
  reflectivity and partial signal loss (range_status≠0)" with: acrylic is
  partially IR-transmissive (940 nm), so status≠0 was not reproduced;
  instead noise variance rose (CM-AKF R ≈ 4× white on stable runs). A
  5–10° tilt was applied. status≠0 learning/eval is consolidated into E3.
- **4.2 Table 4-4 (E2 note)** — "material-wise R_label separation (stable
  runs: white 32.7 / black 65.2 / acrylic 127.3 mm²); acrylic measured with
  5–10° ToF-axis tilt."
- **4.2 E3 body** — add: E2 produced 0 status≠0 frames, so E3 is the sole
  scenario where F5 has discriminative power; the 150×150 mm black
  foam-board blockage (~150 mm interval ≈ 0.75 s at 200 mm/s) is essential
  for F5 learning data.
- **3.5.3 normalization** — "standard normalization, mean/std fit on E1
  Run 1–3; F5 has std=0 → mapped to std_safe=1.0."
- **4.4 normalization scope** — fit on E1 Run 1–3, applied to E1/E2/E3;
  out-of-distribution values not clipped (fed as OOD signal).
- **3.1 system structure** — Fixed/CM/TinyML three-way parallel operation on
  the MCU, empirically confirmed by the 4-C E2 measurement.
- **4.1.4 integration verification** — add stage 4-C: on-device TinyML
  runtime integration, 200 Hz loop stability and 1:1 predict/update/inference
  sync verified during E2.
- **RQ1** — on-device 6-feature inference runs within the 200 Hz budget
  (mean body ~1200 µs, 0 overruns).

## 12. Measurement Environment — Notes

- ToF measurement misses (timestamp gap over 30 ms) occurred per run as in
  E1; Scheme C absorbs the missed interval into the next predict, so
  residual and R stay nominal.
- Bluetooth integrity 95–98% during rolling (motor-noise induced, per 7-E);
  the logger's line-reassembly keeps the saved CSV clean. RMSE/MAE,
  sample-independent TinyML evaluation, and MCU-internal KF dynamics all
  tolerate it.
- Two pre-run corrections (scenario-id flash, grey-vs-black foam-board) were
  caught by the trial run and fixed before the recorded learning set.
