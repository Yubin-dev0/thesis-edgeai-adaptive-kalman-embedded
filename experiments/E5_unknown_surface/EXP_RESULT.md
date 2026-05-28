# E5 Unknown Surface Generalization — Measurement Result Report

| | |
|---|---|
| Scenario | E5 — Unknown Surface Generalization (untrained wall reflectivity) |
| Date | 2026-05-21 |
| Runs | 5 runs (grey foam-board 8-section 5 T) |
| Firmware | 28-column triple-KF (Fixed KF + CM-AKF + TinyML-AKF), Scheme C, stage 4-C |
| Result | **PASS** — 5 runs nominal, TinyML-AKF generalizes to the untrained surface (bounded R, no clamp saturation) |

Corresponds to scenario E5 in thesis Table 4-4 (Section 4.2).

---

## 1. Prerequisites — All Complete

- [x] **E2 learning-set measurement complete** — 9 runs (white/black/acryl
      ×3), `experiments/E2_Reflectivity/` pushed. The TinyML model evaluated
      here was trained on this distribution.
- [x] **Stage 4-C TinyML integration complete** — X-CUBE-AI runtime on the
      MCU, third KF instance `kf_tinyml` fed by on-device TinyML R
      prediction, 28-column CSV. Same build as E2/E3/E4.
- [x] **F5 definition aligned to thesis** — `tof_meas_rate` computed as the
      `range_status==0` ratio over a W=20 sliding window (thesis Table 3-1).
- [x] **Normalization constants hardcoded** — standard normalization
      (mean, std) fit on E1 Run 1–3, embedded in firmware (thesis 3.5.3).
      Grey foam-board values are fed un-clipped as an out-of-distribution
      signal (thesis 4.4 normalization scope).

## 2. Experimental Conditions (Thesis 4.1.2 / 4.2)

| Item | Planned | Measured |
|---|---|---|
| Wall | Grey single-sided foam-board 8-section 5 T (untrained material) | Same |
| Start-to-wall distance | 500 mm (tape measure) | sensor-referenced ~521–530 mm |
| Floor | MDF base board | Same |
| Travel | One-way manual roll, ~100–150 mm/s | Same range |
| Lighting | Direct sunlight blocked, same across runs | Same |
| Number of runs | 5 | **5 collected** |
| Samples per run | 1000 loops ≈ 5 s | Same |
| Valid rows per run | expected ~125 | **measured 189–194** |

> **Run-role note.** E5 is a generalization-test scenario, not a learning
> scenario: the grey foam-board is deliberately excluded from the TinyML
> training set. All 5 runs are evaluation runs against the frozen E2-trained
> model. Per thesis 4.2, E5 is cross-analysed with the E2 evaluation runs
> (transparent acrylic) to quantify in-distribution vs out-of-distribution
> performance.

## 3. Firmware (E5 Measurement — 28-column Triple-KF, stage 4-C)

E5 is measured with the same stage 4-C firmware as E2/E3/E4 — **three** KF
instances run concurrently (`kf_fixed`, `kf_cm`, `kf_tinyml`), all fed the
same ToF/encoder input and logged side by side to a 28-column CSV. The
TinyML-AKF estimate is produced **on the MCU in real time**.

Items applied for the measurement:

- [x] `PHASE6_N_TEST_LOOPS = 1000U` — single rolling run, ~5 s at 200 Hz
      (vs 360000 for the E4 30-min static run).
- [ ] `CSV_SCENARIO_ID = 5U` — **intended, but not reflected in the flashed
      build** (see Section 6). The CSV `scenario_id` column reads 4. Data is
      genuine E5; the field requires a post-hoc 4→5 correction.
- [x] **B1 button trigger** — same as E1/E2 (excludes the stationary
      transient after reset; LD2 blink = "press B1", solid = "measuring").
- [x] **Encoder sign correction** — `int16_t dr = -(enc_r_now - enc_r_prev);`
      single-entry correction (same as E1/E2).
- [x] **Scheme C predict/update 1:1** — unchanged, applied to all three KF
      instances.
- [x] **TinyML inference path** — 6 features (F1–F6) → standard
      normalization → INT8 quantization → `stai_network_run` → INT8
      dequantization → `expm1` (log1p inverse) → clamp [1, 10000] → injected
      as `kf_tinyml.R`. Runs once per ToF update, before predict/update.
- [x] **28-column CSV header** — `tinyml_estimate_mm`, `tinyml_R`,
      `tinyml_infer_us` included. DMA-transmitted.

## 4. TinyML On-Device Inference — Verification

The focus of E5 is whether the on-device TinyML model generalizes to an
untrained surface. Verified during measurement:

**Inference executes.** The `# TinyML infer: count=...` summary line is
present in all 5 runs, with count (275–276) matching the predict/update
count per run. `tinyml_R` varies per step rather than staying at the init
value (24.0).

**Real-time budget (RQ1).** Firmware mean body time stayed in the ~1210 µs
range with the inference path active, within the 200 Hz loop budget
(5000 µs). Per-loop overrun count was 0 in all runs. Mean inference time
34.89–34.92 µs.

**Output behaviour on an untrained surface.** `tinyml_R` stayed bounded
within [1.74, 321.26] across all 5 runs and never reached the clamp limits
(AI_R_MIN=1.0, AI_R_MAX=10000.0). On the four stable runs (run01–04) it
tracked `cm_R` closely (TinyML R mean ≈ 17 vs CM-AKF R mean ≈ 18, ≈ 90–95%),
a tighter coupling than the ~50–80% seen on the trained E2 materials. This
indicates the model did not collapse to a default value nor diverge when fed
out-of-distribution features — the core RQ2/RQ3 evidence.

## 5. Measurement Results (5 Runs)

| Run | Fixed residual mean | Fixed residual σ | CM-AKF R mean | CM R range | TinyML R mean | TinyML R range | roll dist | valid rows |
|---|---|---|---|---|---|---|---|---|
| run01 | −0.73 mm | 4.75 | 16.87 | 2.56 – 60.45 | 15.82 | 2.59 – 51.12 | 428.9 mm | 191 |
| run02 | −1.25 mm | 4.29 | 20.31 | 1.48 – 62.64 | 18.95 | 1.85 – 72.88 | 422.3 mm | 189 |
| run03 | −1.04 mm | 4.31 | 16.43 | 1.67 – 58.65 | 15.29 | 1.74 – 55.33 | 439.2 mm | 194 |
| run04 | −1.37 mm | 4.62 | 19.12 | 1.00 – 65.90 | 18.04 | 1.74 – 75.80 | 431.8 mm | 192 |
| run05 | −2.24 mm | 4.72 | **102.85** | 1.00 – 489.49 | **55.92** | 1.74 – 321.26 | 389.9 mm | 193 |

### Group summary (5 runs combined, init row excluded)

| Metric | Value |
|---|---|
| signal_rate mean | 16.26 MCps |
| status=0 ratio | 100% (all runs) |
| CM-AKF R mean / median | 35.21 / 10.13 |
| TinyML R mean / median | 24.84 / 8.11 |
| TinyML R / CM-AKF R (mean ratio) | 71% |
| TinyML clamp saturation | 0 (neither 1.0 nor 10000.0 reached) |

### Group summary — stable runs only (excluding run05)

| Metric | run01–04 mean |
|---|---|
| CM-AKF R mean | 18.21 |
| TinyML R mean | 17.03 |
| TinyML / CM-AKF | 94% |

- **signal_rate** — grey foam-board mean ≈ 16.3 MCps. Against the E2
  materials (white ≈ 20.5, black ≈ 10.1, acrylic ≈ 13.2), grey sits between
  black and white but **closer to white**, not at a clean midpoint.
- **CM-AKF R** — on stable runs ≈ 18, **lower than every E2 material**
  (E2 stable: white 32.7 / black 65.2 / acrylic 127.3). Grey is an *easier*
  noise environment than the trained set, not a harder/middle one.
- **TinyML R** — follows CM-AKF closely on grey (≈ 94% on stable runs),
  staying bounded and ordered correctly relative to the trained materials.
- **Fixed residual σ** — ≈ 4.5 mm, the lowest across all scenarios,
  consistent with grey being a low-noise surface.

> **Outlier run.** run05 shows 5–6× higher R than its siblings (CM-AKF R
> 102.85 vs ~18, TinyML R 55.92 vs ~17). Its roll distance is the shortest
> (389.9 mm) and it approaches closest to the wall (tof_min 12 mm), the same
> start-offset / hand-roll-wobble signature seen in E2 (white_run03,
> black_run03). It is kept as natural measurement variability; the
> post-processing role should flag it for outlier review. Excluding it
> recovers the clean low-R profile (table above).

## 6. Key Finding — Grey Is Not Mid-Reflectivity + scenario_id Flash Miss

**(a) Reflectivity hypothesis revised.** Thesis 4.2 assumed grey foam-board
would occupy a "mid-reflectivity gap" between the trained white and black
materials. Measurement contradicts this: grey's signal_rate (≈ 16 MCps) and
CM-AKF R (≈ 18 on stable runs) make it a *low-noise* surface, easier than
white (R ≈ 33), black (R ≈ 65) and acrylic (R ≈ 127). The generalization is
therefore tested on the *easy* side of the training distribution, not its
interior. The result is still valid RQ2/RQ3 evidence — the model stays
bounded and correctly ordered on an untrained surface — but the thesis
narrative must drop the "mid-reflectivity" framing.

**(b) status≠0 again absent.** As in E1/E2, `range_status != 0` occurred 0
times across all 5 runs, so F5 (Measurement Rate) is constant 1.000 in E5.
F5 learning/discriminative data remains sourced solely from E3 (sensor
occlusion). Consistent with thesis Table 4-6.

**(c) scenario_id flash miss.** The CSV `scenario_id` column reads **4**, not
5, in all 5 runs. The `PHASE6_N_TEST_LOOPS` change (→1000) was flashed but
the `CSV_SCENARIO_ID = 5U` change was not reflected in the built image. The
data is genuine E5 (filename, 5 s duration, grey-surface signal profile all
confirm it); the field needs a post-hoc 4→5 correction before analysis. The
post-processing role handles this during the per-algorithm split.

## 7. Data-Collection Success Criteria

| Criterion | Value | Note |
|---|---|---|
| Valid travel samples | ≥ 100 | measured 189–194, passes |
| Mean roll speed | 100 – 150 mm/s | E1/E2-consistent range |
| ToF status=0 ratio | ≥ 95% | measured 100% all runs |
| Bluetooth integrity | ≥ 50% (anomaly threshold) | 96.45–98.98% (mean 97.77%), within normal |
| KF predict / update | equal (Scheme C) | 275:275 or 276:276 all runs |
| TinyML inference present | count ≈ predict/update | 275–276 all runs |
| TinyML R bounded | no clamp saturation | 0 saturation events |

## 8. Measurement Procedure (per Run)

Repeated ×5:

1. Mount the grey foam-board wall; re-check the 500 mm start distance.
2. On the PC, run `py logger_cli_debug.py E5 <run_number> [port]`. The logger
   auto-detects the column count from `# CSV_HEADER:` — should show 28.
3. Reset the board → LD2 blinks → press B1 → measurement starts (LD2 solid).
4. Roll the robot toward the wall, steadily and smoothly.
5. Wait for auto-stop at 1000 loops; check the firmware summary. The
   `KF predict / update` counts must be equal (Scheme C). The
   `# TinyML infer: count=...` line should be present with count ≈
   predict/update. If either check fails, re-measure that run.
6. Quick-check `tof_signal_rate` and `cm_R` against the expected direction
   before continuing.

> **Logger header race.** The first attempt at run05 had the logger mis-read
> the header as 53 columns (boot-banner lines merging into the header line),
> invalidating all rows. Re-running the logger after the board had settled
> recovered a clean 28-column capture. The post-processing role should harden
> the header parser to match the `# CSV_HEADER:` prefix explicitly and reject
> candidates whose column count ≠ 28.

## 9. Data Flow

```
one firmware measurement = one CSV  (28 columns: 12 shared + 6 fixed + 7 cm + 3 tinyml)
        │
        ▼  logs_final/   final 4-C firmware logs (E5_run01–05_<timestamp>.csv)
        │
        ▼  CSV logs handed to the post-processing role — per-algorithm split,
           filename normalisation, GT column computation, scenario_id 4→5 fix.
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
- `gt_distance_mm` is written as 0 by the firmware → computed in
  post-processing.
- `scenario_id` reads 4 (flash miss, Section 6c) → corrected to 5 in
  post-processing.
- `tof_meas_rate` holds the F5 ratio (status==0 over W=20); constant 1.000
  in E5.

## 10. Ground Truth (GT)

Same method as E1/E2: dynamic GT = reference distance − cumulative encoder
distance, reference distance = mean ToF over the stationary pre-run segment.
MM_PER_PULSE ≈ 0.05397 mm. The final wall-collision segment is trimmed in
analysis.

## 11. Thesis Revision Items (from E5)

- **4.2 E5 body** — drop the "grey occupies a mid-reflectivity gap"
  framing. Replace with: grey foam-board measured as a low-noise surface
  (signal_rate ≈ 16 MCps, CM-AKF R ≈ 18 on stable runs, below all E2
  materials). E5 therefore tests generalization on the easy side of the
  training distribution. TinyML-AKF stayed bounded (R ∈ [1.74, 321.26], no
  clamp saturation) and tracked CM-AKF at ≈ 94% on stable runs, confirming
  out-of-distribution generalization without divergence.
- **4.2 Table 4-4 (E5 note)** — "grey foam-board 8-section 5 T, untrained
  material; stable-run CM-AKF R ≈ 18 mm² (below white 33 / black 65 /
  acrylic 127); TinyML R ≈ 17 mm² (≈ 94% of CM-AKF)."
- **4.2 E5 cross-analysis** — quantify in-distribution (E2 acrylic eval)
  vs out-of-distribution (E5 grey) performance once GT/RMSE are computed in
  post-processing.
- **4.2 / Table 4-6 F5 note** — E5 produced 0 status≠0 frames, so F5 is
  constant in E5 as in E1/E2; E3 remains the sole F5 learning source.
- **5 (discussion)** — the unexpected grey-vs-white signal ordering shows
  that perceptual "greyness" does not map to ToF reflectivity; this supports
  the multivariate-feature TinyML approach over a single signal-strength
  heuristic (RQ3).
- **RQ1** — on-device 6-feature inference runs within the 200 Hz budget
  (mean body ~1210 µs, 0 overruns, inference ~35 µs).

## 12. Measurement Environment — Notes

- ToF measurement misses (timestamp gap over 30 ms) occurred per run as in
  E1/E2; Scheme C absorbs the missed interval into the next predict, so
  residual and R stay nominal.
- Bluetooth integrity 96–99% during rolling (motor-noise induced); the
  logger's line-reassembly keeps the saved CSV clean. One stray bit error in
  the run03 firmware summary line (`# CSV TX` → `# FSV TX`) is cosmetic and
  did not affect any data row.
- run05 first attempt invalidated by a logger header mis-parse (53 columns);
  re-measured cleanly. See Section 8.
- The `scenario_id` flash miss (Section 6c) was found in post-measurement
  review, not during the runs; all other firmware-summary checks passed live.
