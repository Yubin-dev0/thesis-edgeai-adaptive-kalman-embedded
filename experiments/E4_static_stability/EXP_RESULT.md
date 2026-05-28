# E4 Static Long-Term Stability — Measurement Result Report

| | |
|---|---|
| Scenario | E4 — Static Long-Term Stability (30-min stationary logging, motors OFF) |
| Date | 2026-05-20 |
| Runs | 3 runs × 30 min (white foam-board, stationary) |
| Firmware | 28-column triple-KF (Fixed KF + CM-AKF + TinyML-AKF), Scheme C, stage 4-C, `PHASE6_N_TEST_LOOPS=360000` |
| Result | **PASS** — 3 runs nominal, all checklist items met, R̂ run-to-run CV 0.44% |

Corresponds to scenario E4 in thesis Table 4-4 (Section 4.2).

---

## 1. Prerequisites — All Complete

- [x] **E1 baseline measurement complete** — white foam-board surface
      reference established (`experiments/E1_baseline/`).
- [x] **E2 reflectivity measurement complete** — 9 learning runs, on-device
      TinyML verified (`experiments/E2_Reflectivity/`).
- [x] **E3 dynamic occlusion measurement complete** — F5 learning data
      source (`experiments/E3_DynamicOcclusion/`).
- [x] **Stage 4-C TinyML integration stable** — X-CUBE-AI runtime running
      on the MCU, third KF instance `kf_tinyml` fed by on-device R
      prediction. Same firmware as E2/E3.
- [x] **E4 firmware patch applied** — `CSV_SCENARIO_ID = 4`,
      `PHASE6_N_TEST_LOOPS = 360000` (200 Hz × 1800 s = 30 min),
      `tinyml_infer_us` changed from max-so-far to per-row last value,
      `ai_infer_cycles_sum` widened to `uint64_t` (uint32 overflows at
      ~24 min).

## 2. Experimental Conditions (Thesis 4.1.2 / 4.2)

| Item | Planned | Measured |
|---|---|---|
| Wall | White foam-board 8-section (E1-consistent, surface variable controlled) | Same |
| Start-to-wall distance | 500 mm (tape measure) | sensor-referenced ~522–527 mm |
| Floor | Plywood + MDF base board | Same |
| Travel | **None — robot stationary, motors OFF** | Same (static) |
| Lighting | Direct sunlight blocked, same across runs | Same |
| Number of runs | 3 × 30 min | **3 collected** |
| Samples per run | 360,000 loops = 1800 s @ 200 Hz | Same (360,000 / 360,000) |
| Valid rows per run | expected ~90,000 (50 Hz logging) | **measured 83,747 – 83,883** (≈96% integrity) |

> **Surface-control note.** E4 deliberately uses the **same white foam-board
> as E1** so that the surface (reflectivity) variable is held constant. This
> isolates the long-term factors under test — battery discharge, thermal
> effects, R̂ drift, cumulative inference latency, memory stability — from
> any measurement-noise change due to material. This is the opposite design
> intent from E2 (which varies the surface on purpose).

## 3. Firmware (E4 Measurement — 28-column Triple-KF, stage 4-C, 30-min)

E4 is measured with the same stage 4-C firmware as E2/E3 — **three** KF
instances running concurrently (`kf_fixed`, `kf_cm`, `kf_tinyml`) fed the
same ToF/encoder input, logged to a 28-column CSV. The only differences from
E2 are the run length (360,000 loops) and two instrumentation changes needed
for the long run:

Items applied for the measurement:

- [x] `CSV_SCENARIO_ID = 4U` — E4.
- [x] `PHASE6_N_TEST_LOOPS = 360000U` — 30-min run (vs 1000 for E1/E2/E3).
- [x] **B1 button trigger** — same as E1/E2 (excludes the stationary
      transient after reset; LD2 blink = "press B1" cue, solid = "measuring").
- [x] **`tinyml_infer_us` meaning changed** — max-so-far → per-row **last**
      inference time. Required so each CSV row reflects the instantaneous
      inference latency, enabling the 30-min latency time-series analysis.
      The CSV column name is unchanged, so the post-processing role and
      filename convention are unaffected.
- [x] **`ai_infer_cycles_sum` widened to `uint64_t`** — at 50 Hz × 1800 s ×
      ~63,600 cycles/inference the uint32 accumulator would overflow at
      ~24 min, corrupting the end-of-run mean. uint64 removes this.
- [x] **Scheme C predict/update 1:1** — unchanged. Applied to all three KF
      instances.
- [x] **Encoder sign correction** — `int16_t dr = -(enc_r_now - enc_r_prev);`
      unchanged (motors OFF, so encoder deltas are ≈0 throughout).
- [x] **28-column CSV header** — identical layout to E2/E3. DMA-transmitted.

## 4. TinyML On-Device Inference — Long-Run Verification

E4 is the first scenario to exercise on-device TinyML inference continuously
for 30 minutes. Verified across the 3 runs:

**Inference executes throughout.** `# TinyML infer: count=` reported 98,686 /
98,697 / 98,702 inferences per run (matching the predict/update count),
242,085 cumulative. The `tinyml_R` CSV column varies per step, confirming
the model produces live predictions for the full 30 min.

**Real-time budget (RQ1) — deterministic.** Inference latency held at
**mean 35.00 µs, std 0.007 µs, max 38 µs** across all 242,085 inferences —
effectively deterministic. This is **7% of the 500 µs (90,000-cycle @ 180 MHz)
budget**. Main-loop body time held at **1239 µs mean** (27.5% of the 4500 µs
loop budget) with **0 overruns** across 1,080,000 loops.

**Long-run stability.** No drift in inference time, no memory-related
slowdown, no IWDG reset (36,000 refreshes/run, well within the 8 s timeout).
The uint64 accumulator fix held for the full run.

## 5. Measurement Results (3 Runs × 30 min)

> Statistics computed after a 60 s warm-up exclusion (W=20 window fill +
> R̂ convergence). "Stable region" = `timestamp_ms ≥ 60000`.

| Run | Stable rows | CM-AKF R mean | CM R std | CM R min–max | TinyML R mean | Fixed est mean | Fixed est σ | TinyML infer mean | TinyML infer max |
|---|---|---|---|---|---|---|---|---|---|
| run01 | 83,883 | 22.822 | 8.215 | 2.47 – 68.3 | 23.28 | 522.86 mm | 1.99 mm | 35.00 µs | 38 µs |
| run02 | 83,747 | 23.022 | 8.235 | — | 22.77 | 524.58 mm | 1.77 mm | 35.00 µs | 35 µs |
| run03 | 83,851 | 22.942 | 8.128 | — | ~23.0 | 525.17 mm | 1.74 mm | 35.00 µs | 36 µs |

### Run-to-run summary (reproducibility)

| Metric | Grand mean | Between-run σ | CV / spread |
|---|---|---|---|
| CM-AKF R̂ | 22.929 | 0.101 | **CV 0.44%** |
| Fixed KF estimate | 524.20 mm | 1.20 mm | spread 2.31 mm |
| TinyML inference latency | 35.00 µs | 0.007 µs | effectively deterministic |
| Main-loop body time | 1239 µs | ~0 µs | identical to µs |

### R̂ 30-min drift (per 5-min segment, run01)

| Segment | R̂ mean | Δ from 30-min mean |
|---|---|---|
| 0–5 min | 23.13 | +1.34% |
| 5–10 min | 23.05 | +0.99% |
| 10–15 min | 22.68 | −0.62% |
| 15–20 min | 22.86 | +0.17% |
| 20–25 min | 22.45 | **−1.61%** (max) |
| 25–30 min | 22.87 | +0.22% |

run02 (±1.58%) and run03 (±1.22%) show the same pattern — **no cumulative
drift over 30 min**.

- **CM-AKF R̂ converges to ≈ 22.9 ≈ KF_R_INIT (24).** In a static condition,
  CM-AKF's adaptation has no effect and the filter behaves like Fixed KF.
  This is consistent with the thesis narrative: CM-AKF adaptivity is only
  meaningful in varying environments (E2/E3/E5).
- **TinyML R̂ ≈ CM-AKF R̂** (23.0 vs 22.9). In the static condition the
  TinyML model reproduces CM-AKF's estimate closely — confirming the model
  learned CM-AKF behaviour in-distribution. Any TinyML advantage must appear
  in dynamic conditions (E2/E3/E5).
- **Fixed KF estimate σ ≈ 1.8 mm** on a 525 mm static target. The KF
  flattens the raw ToF noise (σ ≈ 20 mm class) to ±1.8 mm. This is even
  tighter than the E0 simulation √P_ss = 4.42 mm, because the static
  condition has no model-vs-reality mismatch (Q assumption is not violated).

## 6. Key Finding — Hardware Deterministic Behaviour (RQ1 Evidence)

The three runs reproduce identical timing to µs resolution:

- Main-loop body time: 1239 µs across all three runs (between-run σ ≈ 0).
- TinyML inference: mean 35.00 µs, std 0.007 µs across 242,085 inferences.
- 0 loop overruns across 1,080,000 loops; 0 ISR overruns.

This is strong evidence for RQ1 (the embedded system meets hard real-time
constraints) and provides the complete timing dataset for thesis 4.3 —
**no separate TinyML-only timing firmware is needed** (see Section 11).

## 7. Data-Collection Success Criteria

| Criterion | Value | Note |
|---|---|---|
| Full 30-min run | 360,000 loops | 3/3 runs completed, 0 ISR overrun |
| ToF status=0 ratio | ≥ 95% | measured 100% all runs |
| Main-loop mean | ≤ 4.5 ms | measured 1.24 ms (27.5% of budget) |
| R̂ 30-min mean stability | ±5% | measured ±1.61% (5-min segments) |
| TinyML inference budget | ≤ 0.5 ms | measured 35 µs (7% of budget) |
| Bluetooth integrity | < 50% drop (anomaly threshold) | 95.85–95.99%, within normal |

## 8. Measurement Procedure (per Run)

Repeated 3 times:

1. Position the robot stationary at ~500 mm from the white foam-board wall.
   Motors OFF — the robot does not move during the run.
2. On the PC, start the logger. It auto-detects the column count from
   `# CSV_HEADER:` — should show 28.
3. Reset the board → LD2 blinks → press B1 → measurement starts (LD2 solid).
4. Leave the robot undisturbed for the full 30 min.
5. Wait for auto-stop at 360,000 loops; check the firmware summary. The
   `KF predict / update` counts must be equal (Scheme C). The
   `# TinyML infer: count=` line should report ~98,700, and
   `# Body overrun` should be 0. If any check fails, re-measure that run.
6. The logger saves `E4_runXX_YYYYMMDD_HHMMSS.csv`.

> **run02 header note.** On run02 the logger missed the `# CSV_HEADER:`
> line at boot and saved the first row as `col0..col27` with data starting
> at seq=1843 (~37 s in). The first ~37 s were not recorded but the
> remaining stable region is more than sufficient. The header must be
> manually restored before handing run02 to the post-processing role.

## 9. Data Flow

```
one firmware measurement = one CSV  (28 columns: 12 shared + 6 fixed + 7 cm + 3 tinyml)
        │
        ▼  logs_final/   final 4-C firmware logs (E4_run01–03.csv)
        │
        ▼  CSV logs handed to the post-processing role — per-algorithm split,
           filename normalisation, GT column computation are done there.
           (TinyML R is on-device, no PC post-hoc inference needed.)
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
- `gt_distance_mm` is written as 0 by the firmware. For E4 (static, motors
  OFF) the true distance is constant ~500 mm; GT is the stationary reference
  distance, computed in post-processing.
- `tinyml_infer_us` for E4 holds the **per-row last** inference time (not the
  E2/E3 max-so-far), enabling the latency time-series analysis.

## 10. Ground Truth (GT)

E4 is static (motors OFF), so the true wall distance is constant. GT = the
stationary reference distance (mean ToF over the run, ~525 mm sensor-
referenced). Unlike E1/E2, there is no cumulative-encoder term because the
robot does not move. The encoder distance stays ≈0 throughout (verified:
`encoder_distance_mm` ≈ 0).

## 11. Thesis Revision Items (from E4)

- **4.2 Table 4-4 (E4 row)** — add the wall material, currently blank:
  "robot stationary (motors OFF), **white foam-board 8-section** wall at
  500 mm, 30-min continuous static logging; battery ADC monitoring."
- **4.2 E4 body** — add one sentence: "The wall surface uses the same white
  foam-board as E1 to control the surface variable, isolating long-term
  factors (battery discharge, thermal effects, cumulative error) for
  evaluation."
- **4.3 Table 4-5 (inference time)** — replace the planned method
  ("TinyML-only firmware, dummy-feature 100-inference average; full-
  integration timing measured separately in the Chapter 5 live validation")
  with: "measured on the full-integration firmware (E4 static stability,
  3 runs × 30 min) over 242,085 real-feature inferences via DWT cycle
  counter; mean 35 µs (std 0.007 µs), max 38 µs — 7% of the 500 µs budget."
- **4.1.4 integration verification** — add E4 long-run result: 30-min
  continuous on-device inference, deterministic timing (loop 1239 µs,
  inference 35 µs), 0 overruns, no IWDG reset, uint64 accumulator stable.
- **Chapter 5 (stability discussion)** — CM-AKF R̂ converges to ≈ KF_R_INIT
  in static conditions (adaptation inactive); TinyML R̂ reproduces CM-AKF
  in-distribution; run-to-run R̂ CV 0.44% demonstrates reproducibility.
- **RQ1** — fully answered by E4: on-device 6-feature inference is
  deterministic (35.00 µs ± 0.007) and the 200 Hz loop holds (1239 µs, 0
  overruns) over 30 min × 3.

## 12. Measurement Environment — Notes

- **Bluetooth integrity 95.85–95.99%** across all 3 runs, **even with motors
  OFF** — this isolates the loss to the HC-06 module's 30-min continuous-
  transmission characteristic (not motor noise as in the rolling scenarios).
  Firmware-side `drops=0`, `seq=90000` confirms the MCU transmitted all
  90,000 frames; the loss is in the wireless link. The seq field allows
  time-axis reconstruction, so RMSE/R̂ analysis is unaffected (3-run R̂ CV
  0.44% confirms this).
- ToF measurement misses (timestamp gap over ~20 ms) occur as in E1/E2;
  Scheme C absorbs the missed interval into the next predict, so residual
  and R stay nominal. In the static condition this has negligible effect
  since there is no displacement to mispredict.
- run02 logger header was missed at boot (saved as `col0..col27`, data from
  seq=1843); corrected manually before post-processing handoff.
- The 3 runs were measured back-to-back (90 min total); battery monitoring
  via ADC1 IN4 (PA4) ran throughout for the long-term discharge record.
