# E3 Dynamic Occlusion — Measurement Result Report

| | |
|---|---|
| Scenario | E3 — Dynamic Occlusion (transient ToF beam interference) |
| Date | 2026-05-20 |
| Runs | 5 learning runs + 5 evaluation runs |
| Firmware | 28-column triple-KF (Fixed KF + CM-AKF + TinyML-AKF), Scheme C, stage 4-C |
| Result | **PASS** — occlusion pattern reproduced across all runs, stuck-sensor limit confirmed |

Corresponds to scenario E3 in thesis Table 4-4 (Section 4.2).

---

## 1. Prerequisites — All Complete

- [x] **E1 baseline + E2 reflectivity measurement complete** — E1 runs 1–5
      and E2 9 learning runs pushed.
- [x] **Stage 4-C TinyML integration complete** — X-CUBE-AI runtime
      integrated and running on the MCU. Third KF instance `kf_tinyml`
      fed by on-device TinyML R prediction. CSV 28 columns.
- [x] **F5 definition aligned to thesis** — `tof_meas_rate` computed as the
      ratio of `range_status==0` within a W=20 sliding window
      (thesis Table 3-1).
- [x] **Scenario mechanism redesigned** — original static black-foam-board
      side-blockage replaced with dynamic aluminium-panel occlusion, after
      pre-trials confirmed black foam board does not induce `range_status≠0`
      (see Section 6).

## 2. Experimental Conditions (Thesis 4.1.2 / 4.2)

| Item | Planned (original) | Measured (redesigned) |
|---|---|---|
| Wall | White foam-board 8-section | Same |
| Start-to-wall distance | 500 mm (tape measure) | sensor-referenced, E1 R0-consistent |
| Floor | Plywood + MDF base board | Same |
| Occluder material | 150×150 mm black foam board, static | **A4 aluminium panel (protective film on), hand-held** |
| Occluder mounting | side, fixed at 250 mm | **dynamic insertion into beam at 100 mm point** |
| Occlusion trigger | robot reaching 250 mm | robot passing the 250 mm tape mark |
| Occlusion hold | ~0.75 s (150 mm interval @ 200 mm/s) | ~0.75–1 s (operator-controlled) |
| Travel | One-way manual roll, ~200 mm/s | Same |
| Lighting | Direct sunlight blocked, same across runs | Same |
| Number of runs | 5 (run1–5) | **5 learning + 5 evaluation** |
| Samples per run | 1000 loops ≈ 5 s | Same |
| Valid rows per run | expected ~125 | **measured 226–235** |

> **Run-split note.** Per thesis Table 4-6, E3 run1–3 are the TinyML
> learning set and run4–5 the evaluation set. Learning runs were measured
> on the 4-B-2 firmware (Fixed/CM only); evaluation runs were measured on
> the frozen 4-C firmware (on-device TinyML), so the two sets are recorded
> with distinct firmware versions and timestamps (see Section 9).

## 3. Firmware (E3 Measurement — 28-column Triple-KF, stage 4-C)

E3 evaluation is measured with the stage 4-C firmware that runs **three**
KF instances concurrently — `kf_fixed` (fixed R), `kf_cm` (CM-AKF), and
`kf_tinyml` (R injected from on-device TinyML inference) — all fed the same
ToF/encoder input, logged side by side to a 28-column CSV. The TinyML-AKF
estimate is produced **on the MCU in real time**.

Items applied for the measurement:

- [x] `CSV_SCENARIO_ID = 3U` — E3.
- [x] **Banner/result labels** — "E3 Dynamic Occlusion".
- [x] **B1 button trigger** — same as E1/E2 (excludes the stationary
      transient after reset).
- [x] **Encoder sign correction** — `int16_t dr = -(enc_r_now - enc_r_prev);`
      single-entry correction (same as E1/E2).
- [x] **Scheme C predict/update 1:1** — unchanged; applied to all three KF
      instances.
- [x] **TinyML inference path** — 6 features (F1–F6) → standard
      normalization → INT8 quantization → `stai_network_run` → INT8
      dequantization → `expm1` (log1p inverse) → clamp [1, 10000] → injected
      as `kf_tinyml.R`. Runs once per ToF update, just before predict/update.
- [x] **28-column CSV header** — adds `tinyml_estimate_mm`, `tinyml_R`,
      `tinyml_infer_us`. DMA-transmitted.

> Only the four label/scenario-id lines differ from the E2 firmware
> (`CSV_SCENARIO_ID`, boot banner, result header, result footer). The KF
> algorithm code, Scheme C structure, F5 window (W=20), and predict/update
> 1:1 are unchanged, so E0/Phase 6 equivalence is preserved.

## 4. Dynamic Occlusion — Setup and Procedure

The occlusion is produced by an operator manually inserting an A4 aluminium
panel into the ToF beam path while a second hand rolls the robot.

**Physical setup**

| Item | Value |
|---|---|
| Wall | White foam-board, 500 mm from start point |
| Tape marks | 500 mm (start), 250 mm (trigger), 100 mm (occluder position) |
| Occluder | A4 aluminium panel, protective film on |
| Occluder orientation | perpendicular to the ToF beam (specular return) |
| Occluder insertion point | 100 mm from the wall |
| Hold time | ~0.75–1 s |
| Trigger | robot passing the 250 mm tape mark |
| Travel | one-way manual roll toward the wall, ~200 mm/s |

**Procedure (per run)**

1. Align the robot at the 500 mm start mark.
2. On the PC, run `py logger_cli.py E3 <run_number> [port]`. The logger
   auto-detects the column count from `# CSV_HEADER:` — should show 28.
3. Reset the board → LD2 blinks → press B1 → measurement starts (LD2 solid).
4. Roll the robot toward the wall, steadily and smoothly (~200 mm/s).
5. The moment the robot visually passes the 250 mm tape mark, insert the
   aluminium panel into the ToF beam at the 100 mm point (panel perpendicular
   to the beam, holding only the panel edge so the hand stays out of the beam).
6. Hold ~0.75–1 s, then withdraw.
7. Wait for auto-stop at 1000 loops; check the firmware summary. The
   `KF predict / update` counts must be equal (Scheme C). The
   `# TinyML infer: count=...` line should be present (4-C eval runs).
8. Confirm the occlusion appears in the CSV (measured distance jumps from
   ~250 mm toward the panel distance ~100–200 mm). If absent or shorter than
   ~20 frames, re-measure.

> The original static black-foam-board side-blockage was abandoned after
> pre-trials (Section 6). Manual occlusion was originally excluded in the
> thesis for reproducibility; this constraint is revised (Section 11) since
> dynamic occlusion is the only mechanism that reproduces the intended
> challenge on this hardware.

## 5. Measurement Results

### 5-1. Learning runs (run01–05, 4-B-2 firmware, Fixed/CM only)

| Run | Valid rows | status≠0 | Residual σ (mm) | Residual min (mm) | Occlusion frames | CM R max |
|---|---|---|---|---|---|---|
| E3_run01 | 229 | 0 | 45.25 | −91.63 | 48 (~0.87 s) | 10000 (clamp) |
| E3_run02 | 235 | 0 | 49.80 | −105.87 | 54 (~0.98 s) | 10000 (clamp) |
| E3_run03 | 234 | 0 | 38.45 | −84.21 | 50 (~0.91 s) | 6956.3 |
| E3_run04 | 226 | 0 | 53.42 | −114.31 | 71 (~1.29 s) | 10000 (clamp) |
| E3_run05 | 234 | 0 | 47.12 | −126.98 | 38 (~0.69 s) | 10000 (clamp) |

CM-AKF final estimates (firmware summary):

| Run | Fixed final (mm) | CM final (mm) | CM final P | CM final R |
|---|---|---|---|---|
| E3_run01 | 77.80 | 156.36 | 54.39 | 7034.97 |
| E3_run02 | 101.65 | 189.97 | 58.59 | 8572.32 |
| E3_run03 | 87.87 | 163.67 | 51.15 | 5636.15 |
| E3_run04 | 91.44 | 154.06 | 63.55 | 4570.72 |
| E3_run05 | 73.79 | 143.15 | 45.24 | 6059.96 |

### 5-2. Evaluation runs (run01–05, 4-C firmware, triple-KF on-device)

| Run | Valid rows | status≠0 | Residual σ (mm) | Residual min (mm) | Occlusion frames | CM R max | TinyML R mean | TinyML R max |
|---|---|---|---|---|---|---|---|---|
| E3_run01 | 235 | 0 | 51.30 | −103.68 | 66 | 10000 | 2286.64 | 10000 |
| E3_run02 | 228 | 0 | 44.22 | −110.14 | 39 | 10000 | 1802.89 | 10000 |
| E3_run03 | 226 | 0 | 57.14 | −150.44 | 48 | 10000 | 2104.86 | 10000 |
| E3_run04 | 231 | 0 | 56.62 | −149.97 | 46 | 10000 | 2138.02 | 10000 |
| E3_run05 | 232 | 0 | 60.90 | −193.01 | 89 | 10000 | 1889.86 | 10000 |

### 5-3. Observations

- **Occlusion reproduced in every run** — measured distance jumps from the
  true wall distance (~250 mm) toward the aluminium panel distance
  (~100–200 mm) for ~0.7–1.3 s, while `range_status` stays 0.
- **Stuck-sensor limit confirmed (RQ2)** — `cm_R` reaches the 10000 clamp in
  4/5 learning runs and 5/5 evaluation runs. Residual stays persistently
  large (−85 to −193 mm), and CM-AKF's reactive R adaptation lags: P
  accumulates to 45–64 (vs nominal P_ss = 19.51) and recovery is delayed
  after the occluder is withdrawn.
- **TinyML on-device R adaptation** — `tinyml_R` also reaches the 10000 clamp
  during occlusion and decays gradually after withdrawal (10000 → 9033 →
  8042 → 4325 …), matching the trained recovery behaviour. This is distinct
  from Fixed KF (R = 24 constant) and confirms the three algorithms behave
  differently on the same input (thesis 3.1 fair-comparison premise).
- **signal_rate variation** — occlusion-interval `signal_rate` varies run to
  run (5–27 MCps) due to small differences in aluminium-panel angle. This is
  beneficial as learning-data diversity: the model learns the
  measurement-jump pattern is independent of the exact signal level.

## 6. Key Finding — Black Foam Board Does Not Induce status≠0

The original E3 design assumed a static black-foam-board side-blockage would
produce `range_status≠0` over the blockage interval. Pre-trials disproved
this.

**Pre-trial sequence (7 trials, discarded — kept for the redesign record).**

| Trial | Setup change | Result |
|---|---|---|
| run80 | black foam, side 50 mm, position 100–250 mm | status≠0 = 0; signal_rate monotonic with distance |
| run81 | + occluder height boosted to ToF-axis | identical to run80 |
| run82 | side gap reduced to 20 mm | identical |
| run83 | occluder moved to 250–400 mm (thesis intent) | identical |
| run84 | Step A — black foam held in front of ToF, static | measured 123–148 mm, **status = 0** (reflective surface, not blockage) |
| run85 | side blockage repeated | identical |
| run86 | aluminium **wall** swap | signal_rate ~0.5 MCps, status 0, large variance |
| run87 | aluminium **hand occlusion** (front) | **occlusion reproduced**: distance jump, cm_R → 10000 clamp |

**Cause.** The VL53L0X uses a 940 nm IR source. Black foam board is a
visible-light absorber but only a partial IR absorber, so the beam returns a
(reflected) reading rather than failing — the occluder is read as a
*reflective surface*, not a blockage. Side blockage at the 50/20 mm gap never
intersected the beam (FOV 25° beam edge only grazes the occluder; the signal
is still recovered). This matches E2, where black foam likewise produced 0
status≠0 frames.

**Resolution.** An A4 aluminium panel inserted *frontally* into the beam
reflects strongly enough that the measured distance jumps to the panel
distance, producing a persistent residual outlier — the intended
stuck-sensor challenge — while `status` stays 0. The mechanism therefore
changed from "signal loss (status≠0)" to "measurement-jump outlier", which
is functionally equivalent for the RQ2/RQ3 challenge (both present the KF
with a persistently wrong measurement that residual statistics alone cannot
disambiguate).

**Consequence for F5.** With status≠0 = 0 across E1, E2 and E3, **F5
(Measurement Rate) is constant 1.000 in all scenarios**. F5 carries no
discriminative information in the current dataset; it is retained as a
reserve channel for future signal-loss conditions and documented as a data
limitation (see Section 11).

## 7. Data-Collection Success Criteria

| Criterion | Value | Note |
|---|---|---|
| Valid travel samples | ≥ 100 | measured 226–235, passes |
| Mean roll speed | ~200 mm/s | E-series consistent |
| Occlusion reproduced | distance jump present | confirmed all runs |
| Occlusion interval | ~0.7–1.3 s | 38–89 frames |
| Stuck-sensor pattern | cm_R → clamp | confirmed (9/10 runs reach clamp) |
| Bluetooth drop | < 50% (anomaly threshold) | 95–98% integrity, within normal |

## 8. TinyML On-Device Inference — Verification (Evaluation Runs)

- **Inference executes.** `# TinyML infer: count=...` present, count matching
  predict/update. `tinyml_R` varies per step (not stuck at init 24.0).
- **Real-time budget (RQ1).** Mean body time ~1200 µs with inference active,
  within the 200 Hz loop budget (5000 µs). Per-loop overrun count 0.
- **Output behaviour.** `tinyml_R` tracks the occlusion event (clamp during
  occlusion, gradual decay after), confirming the trained model reacts to the
  measurement-jump challenge on-device.

## 9. Data Flow and File Layout

```
one firmware measurement = one CSV  (28 columns: 12 shared + 6 fixed + 7 cm + 3 tinyml)
        │
        ▼  logs/         learning-set logs (4-B-2 firmware, Fixed/CM only, 17:21)
        │
        ▼  logs_final/   evaluation logs (4-C firmware, on-device TinyML, 21:55)
        │
        ▼  CSV logs handed to the post-processing role — per-algorithm split,
           filename normalisation, GT column computation are done there.
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

- `gt_distance_mm` is written as 0 by the firmware → computed in
  post-processing (`gt = reference_distance − cumulative encoder distance`).
- `tof_meas_rate` holds the F5 ratio (status==0 over W=20), constant 1.000
  in E3.
- **Learning vs evaluation must not be mixed.** Learning runs (4-B-2,
  Fixed/CM only) and evaluation runs (4-C, on-device TinyML) are distinct
  firmware versions. Using the TinyML-bearing evaluation CSVs as training
  data would cause data leakage (the model learning from its own output).

## 10. Ground Truth (GT)

Same method as E1/E2: dynamic GT = reference distance − cumulative encoder
distance, reference distance = mean ToF over the stationary pre-run segment.
MM_PER_PULSE ≈ 0.05397 mm. The final wall-collision segment is trimmed in
analysis. The occlusion interval is identified by encoder distance for
per-interval RMSE reporting.

## 11. Thesis Revision Items (from E3)

- **4.2 Table 4-4 (E3 row)** — replace "side 150×150 mm black foam board,
  ~0.5 s range_status≠0" with: "robot reaching 250 mm triggers operator
  insertion of an A4 aluminium panel at the 100 mm point for ~0.75–1 s;
  status stays 0 while measured distance jumps to ~100–200 mm (outlier)."
- **4.2 E3 body** — rewrite from static black-foam-board occlusion to dynamic
  aluminium-panel occlusion; state the 940 nm IR partial-reflection finding
  for black foam, and that the measurement-jump outlier reproduces the
  2.2.3 CM-AKF stuck-sensor limit. Remove the "manual occlusion not adopted"
  sentence; replace with the redesign rationale.
- **5.x (new) — scenario redesign record** — document the 7 pre-trials, the
  black-foam IR-reflection finding, and the static→dynamic transition as a
  hardware-in-the-loop discovery (honest-limitation principle).
- **5.x (new) — RQ2 evidence** — cm_R reaching the 10000 clamp and delayed
  recovery as the empirical stuck-sensor limit; Fixed vs CM vs TinyML
  divergence on the same input.
- **5.x (new) — reproducibility limit** — operator-timing variation
  (occlusion 38–89 frames, entry 213–273 mm); per-interval RMSE reported by
  encoder-distance window; single operator, pre-agreed manual.
- **5.x (new) — F5 limitation** — status≠0 = 0 across all scenarios → F5
  constant 1.000; retained as reserve channel; 6-feature vs 5-feature
  ablation difference (if any) attributable to data limitation, not model.
- **3.1 system structure** — Fixed/CM/TinyML three-way parallel operation on
  the MCU, empirically confirmed by the 4-C E3 evaluation measurement.
- **4.1.4 integration verification** — stage 4-C on-device TinyML runtime,
  200 Hz loop stability and 1:1 predict/update/inference sync verified during
  E3.
- **RQ1** — on-device 6-feature inference runs within the 200 Hz budget
  (mean body ~1200 µs, 0 overruns) under the occlusion challenge.

## 12. Measurement Environment — Notes

- ToF measurement misses (timestamp gap over 30 ms) occurred per run as in
  E1/E2; Scheme C absorbs the missed interval into the next predict.
- Bluetooth integrity 95–98% during rolling (motor-noise induced); the
  logger's line-reassembly keeps the saved CSV clean.
- Occlusion timing is operator-controlled and therefore varies run to run;
  this variability is recorded as a reproducibility limit (Section 11) and is
  handled in analysis by encoder-distance-based interval identification.
- A trial run on the 4-C firmware (run88) preceded the evaluation set and
  confirmed the E3 label/scenario-id flash and the occlusion pattern before
  the recorded runs.
