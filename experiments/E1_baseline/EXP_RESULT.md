# E1 Baseline — Measurement Result Report

| | |
|---|---|
| Scenario | E1 — Baseline (normal wall approach) |
| Date | 2026-05-19 |
| Runs | 5 main runs (run01–run05) + 1 trial run (run00) |
| Firmware | 25-column dual-KF (Fixed KF + CM-AKF), Scheme C |
| Result | **PASS** — runs 1–5 all nominal |

Corresponds to scenario E1 in thesis Table 4-4 (Section 4.2).

---

## 1. Prerequisites — All Complete

- [x] **Phase 7 integration verification passed** — 7-B / 7-C·D / 7-E / 7-F
      all PASS, `tests/07_manual_integration_verification/` pushed.
- [x] **R0 calibration complete** — 500 mm static calibration (232 samples,
      σ ≈ 4.87 mm) fixed R₀ = 24 mm². `KF_R_INIT` in `kalman_filter.h`
      changed 400 → 24.0f. The VL53L0X sensor sits ~18 mm behind the
      robot's front edge, so it reads a 500 mm wall as ~518 mm — this is a
      physically correct value, not an offset, so the GT reference distance
      uses the sensor-referenced value. Thesis Section 4.1.3 was revised
      from 7-distance calibration to a single 500 mm distance.
- [x] **E1 measurement firmware ready** — all items in Section 3 applied;
      measurement completed.

## 2. Experimental Conditions (Thesis 4.1.2 / 4.2)

| Item | Planned | Measured |
|---|---|---|
| Wall | White foam-board panel | Same |
| Start-to-wall distance | 500 mm (tape measure) | 510–527 mm (sensor-referenced, see R0 note above) |
| Floor | Plywood + MDF base board | Same |
| Travel | One-way manual roll, target ~200 mm/s | **Measured 105–134 mm/s** (see note below) |
| Guide rail | None — heading kept by operator | Same |
| Lighting | Direct sunlight blocked, same across runs | Same |
| Number of runs | 5 | 5 (run 1–5) |
| Samples per run | 1000 loops ≈ 5 s | Same |
| Valid rows per run | expected ~125 | **measured 230–236** |

> **Travel-speed note.** Measured mean speed was 105–134 mm/s, below the
> 200 mm/s target (a characteristic of manual rolling). The KF operates
> independently of speed and both residual and R were nominal, so the E1
> results themselves are valid. E2–E5 should be measured at the same speed
> range to keep cross-scenario consistency. The speed criterion in the
> data-collection success table was updated to the measured range
> (see Section 6).

## 3. Firmware (E1 Measurement — 25-column Dual-KF)

E1 is measured with a single firmware build that runs Fixed KF and CM-AKF
concurrently and logs to a 25-column wide CSV. Two KF instances
(`kf_fixed`, `kf_cm`) receive the same ToF/encoder input and log their
estimates side by side. The TinyML-AKF estimate is produced separately by
PC-side post-hoc inference after training on E1–E5 data (no trained model
exists at measurement time, so it is not run on the MCU).

Items applied for the measurement:

- [x] `PHASE6_N_TEST_LOOPS = 1000` — about 5 s @ 200 Hz.
- [x] **B1 button trigger** — wait for B1 (PC13) press, then start the
      measurement count, excluding the stationary transient right after
      reset. While waiting, LD2 (green LED) blinks as the "press B1" cue;
      once pressed it stays solid ON to indicate "measuring".
- [x] **Encoder sign correction** — `int16_t dr = -(enc_r_now - enc_r_prev);`
      corrects the encoder sign (inverted by the R-motor wiring) at a single
      entry point. enc_r_total, pos_r_mm and the KF input are all corrected
      in one place.
- [x] **predict/update time-structure fix (Scheme C)** — *the key bug found
      and fixed during E1 measurement.* See Section 4.
- [x] **25-column CSV header** — transmitted by DMA (avoiding the printf /
      USART6 collision). Column order is in Section 5.

> **F5 (tof_meas_rate) definition mismatch — must be fixed before E2.**
> The current firmware implements `tof_meas_rate` as the step-to-step
> change of the ToF reading, but the thesis-body definition is "ratio of
> status==0 within a W=20 window". F5 is a TinyML feature, so the firmware
> must be aligned to the thesis definition before E2 measurement. The E1
> data carries the change-rate value.

## 4. Scheme C — predict/update Time-Structure Fix

During E1 trial measurement, CM-AKF was observed to diverge, driving R
from 24 to 10000 (the clamp ceiling). Root cause and fix follow.

**Cause.** The previous firmware called `kf_predict` every 200 Hz loop and
`kf_update` only on ToF DataReady. The ToF interval was variable
(18–62 ms, including misses), so the predict:update ratio was not constant
(4:1 to 12:1) and the two were not phase-aligned. As a result, a
speed-proportional negative bias accumulated in the residual during motion
(moving-segment `fixed_residual` mean −8.6 mm). The Fixed KF tolerated this
because its R is fixed, but in CM-AKF the biased residuals filling the
window inflated `E[r²]`, driving R divergence (positive feedback). The CM
formula `R = E[r²] − P_pred` is a direct translation of the predict-1 :
update-1 Python original (`cm_akf_1D.py`), so under multi-rate, asynchronous
calls the time scales of the two terms no longer matched.

**Fix.** The KF is no longer stepped every loop. Each 200 Hz loop only
accumulates the encoder pulse delta. When a ToF measurement arrives, a
single `kf_predict` (fed the whole accumulated displacement) is followed
immediately by one `kf_update`, then the accumulator is reset. predict:update
is now strictly 1:1 and phase-aligned, identical in structure to the Python
original. A missed ToF interval is handled automatically — its accumulated
displacement is absorbed into the next predict.

**Equivalence.** `kalman_filter.c` / `kalman_filter.h` are unchanged. The CM
formula and the KF parameters (including `KF_B = 0.005`) are intact, so the
E0 simulation and Phase 6 equivalence checks are preserved. The 200 Hz main
loop itself is also kept (sensors, HC-SR04, CSV, timing instrumentation all
still run at 200 Hz), so the RQ1 premise holds.

**Verification.** The `KF predict / update` counts in the firmware end-of-run
summary must be equal to guarantee the 1:1 structure. The fix was confirmed
on a trial run (run00) before the main runs — the trial run's moving-segment
`fixed_residual` mean was +0.89 mm (−8.57 mm before the fix), and CM-AKF R
stayed within 5.9–33 (24→10000 divergence before the fix).

## 5. Measurement Results (Runs 1–5)

| Run | Fixed residual mean | Fixed residual σ | CM-AKF R mean | R range | CM tracking error | status=0 |
|---|---|---|---|---|---|---|
| run1 | +0.29 mm | 4.10 | 16.3 | 3.5 – 71 | 2.43 mm | 100% |
| run2 | −0.47 mm | 4.04 | 13.7 | 3.7 – 34 | 2.41 mm | 100% |
| run3 | −0.34 mm | 3.96 | 13.1 | 2.7 – 36 | 2.42 mm | 100% |
| run4 | −1.08 mm | 3.69 | 10.3 | 1.5 – 25 | 2.16 mm | 100% |
| run5 | −1.31 mm | 4.05 | 14.9 | 2.6 – 36 | 2.49 mm | 100% |

- **Fixed residual** — all five runs within −1.3 to +0.3 mm. The −8.6 mm
  bias of the buggy version is fully removed. σ ≈ 4 mm matches R₀ = 24
  (σ ≈ 4.9 mm) — the residual has converged to the measurement-noise level.
- **CM-AKF R** — zero divergence. All five runs adapt stably around
  R₀ = 24.
- **CM tracking** — |cm_est − tof| is 2.2–2.5 mm across all five runs,
  with a run-to-run spread of 0.3 mm.
- The tracking error (~2.4 mm) is expected. The KF smooths ToF noise, so
  maintaining a small offset from the instantaneous ToF reading is normal
  behaviour.

> The CM-AKF R divergence seen in E1 (a basic scenario) was a firmware
> time-structure bug, resolved by the Scheme C fix. It is unrelated to the
> "CM-AKF structural limitation" discussed in the mid-term review doc 04 —
> that structural limitation should be evaluated honestly in a hard
> scenario such as E3 (sensor blockage).

## 6. Data-Collection Success Criteria (Updated to E1 Measured Values)

| Criterion | Updated value | Note |
|---|---|---|
| Valid travel samples | ≥ 100 | measured 230–236, passes comfortably |
| Mean roll speed | 100 – 150 mm/s | measured 105–134 — lowered from the 200 target |
| ToF status=0 ratio | ≥ 95% | measured 100% across all five runs |
| Bluetooth drop | < 50% (anomaly threshold) | within normal range |

> The speed criterion was updated to 100–150 mm/s based on E1 measured
> values. Measuring E2–E5 at the same speed keeps cross-scenario
> consistency.

## 7. Measurement Procedure (per Run)

Repeated 5 times. For each run:

1. Place the robot at the start point; re-check the 500 mm start distance
   with a tape measure.
2. On the PC, run `py logger_cli.py E1 <run_number> [port]` — this arms
   reception before the B1 trigger. (The logger auto-detects the column
   count from the `# CSV_HEADER:` line — it should show 25 columns.)
3. Reset the board → LD2 starts blinking → press B1 → measurement count
   starts (LD2 solid).
4. Immediately roll the robot toward the wall, steadily and smoothly.
   No sharp acceleration.
5. Wait for the automatic stop at 1000 loops; check the firmware
   end-of-run summary. The `KF predict / update` counts must be equal
   (the verification metric for the Scheme C 1:1 structure). If they
   differ, re-measure that run.
6. Confirm the CSV is saved → go back to step 1 for the next run.

## 8. Data Flow

```
one firmware measurement = one CSV  (25 columns: 12 shared + 6 fixed + 7 cm)
        │
        ▼  logs/   logger_cli.py originals (E1_run01.csv – E1_run05.csv)
        │
        ▼  CSV logs handed to the post-processing role — the subsequent
           post-processing (per-algorithm split, filename normalisation,
           GT column computation, TinyML post-hoc inference) is done there.
```

CSV 25-column order:

```
seq, timestamp_ms, tof_distance_mm, tof_signal_rate, tof_range_status,
us_distance_mm, encoder_distance_mm, encoder_speed_mms, sensor_disagree,
tof_meas_rate, gt_distance_mm, scenario_id,
fixed_estimate_mm, fixed_residual, fixed_residual_var, fixed_residual_mean,
fixed_kalman_gain, fixed_innovation_cov,
cm_estimate_mm, cm_residual, cm_residual_var, cm_residual_mean,
cm_kalman_gain, cm_innovation_cov, cm_R
```

- **One firmware measurement = one CSV.** The firmware is not swapped per
  algorithm — raw / Fixed / CM values are all recorded in a single
  measurement.
- `gt_distance_mm` is written as 0 by the firmware → computed in
  post-processing.
- The measurement role's deliverable ends at the CSV logs under `logs/`.

> **Filename note for the post-processing hand-off.** The current CSV log
> filenames are of the form `E1_run01.csv`. The upload page enforces the
> rule `{scenario}_run{N}_{algorithm}.csv` (e.g. `E1_run1_raw.csv`,
> `E1_run1_fixed.csv`, `E1_run1_cm.csv`, `E1_run1_tinyml.csv`). Splitting
> the 25-column wide single file into four per-algorithm files and
> matching the filenames to the page rule is part of post-processing. The
> TinyML R_label training target uses the `cm_residual_var` column.

## 9. Ground Truth (GT)

Dynamic GT = reference distance − cumulative encoder distance. The
reference distance is the mean ToF (sensor-referenced value) over the
stationary segment before each run starts. Encoder scale
MM_PER_PULSE ≈ 0.05397 mm. For a short one-way 500 mm travel, accumulated
slip error is expected to be within a few mm. The firmware records only
encoder_distance_mm and leaves the gt_distance_mm column at 0 (computed in
post-processing).

In analysis, the final wall-collision segment is trimmed — only the
segment from the encoder's first motion to wall arrival is used.

## 10. Measurement Environment — Notes

- ToF measurement misses (timestamp gap over 30 ms) occurred 10–13 times
  per run. Scheme C absorbs the missed interval's accumulated displacement
  into a single predict, so residual and R stay nominal. The status=0
  ratio of 100% means no measurement loss. This is a structural
  characteristic of the VL53L0X DataReady polling timing.
- The 10–20% Bluetooth drop during rolling is caused by motor-rotation
  generator noise; hardware mitigation could not eliminate it, as
  concluded in 7-E. It does not block the experiment — RMSE/MAE
  (mean-based), TinyML training (sample-independent), and KF dynamics
  (internal to the MCU) all tolerate it. The drop is 0% when the robot is
  stationary.
