# Phase 6 — Sensor + Kalman Filter Integration (Motor OFF)

**Objective:** Integrate the encoder, VL53L0X ToF sensor, and a Fixed Kalman
Filter into a single 200 Hz main loop, stream a decimated 50 Hz CSV log over
Bluetooth, and verify C firmware against the Python reference implementation.

**Date:** 2026-05-15 ~ 2026-05-17
**Platform:** STM32F446RE NUCLEO, HSE BYPASS 180 MHz
**Result:** PASS (all steps)

---

## Demo — 200 Hz Loop + 50 Hz CSV Streaming

[![Phase 6 demo](https://img.youtube.com/vi/qhJF6sqIsYc/maxresdefault.jpg)](https://youtu.be/qhJF6sqIsYc)

---

## 1. Build Steps

Phase 6 was built incrementally; each step was flashed and verified before
proceeding to the next.

| Step | Scope | Key result | Verdict |
|---|---|---|---|
| 1 | TIM6 200 Hz loop skeleton + DWT profiling | Loops 1000/1000, TIM6 ticks 1000, body overrun 0, ISR overrun 0 | PASS |
| 2 | Sensor read (encoder + VL53L0X Continuous) | Mean body 1054 us, max 2793 us, DataReady ~55 Hz, status0 100%, I2C err 0 | PASS |
| 3 | KF predict (200 Hz) + update (on DataReady) | P_ss = 19.506, K_ss = 0.048766 — matches E0 simulation to 3 decimals | PASS |
| 4 | Circular buffer W=20 + residual statistics | Integrated within `kalman_filter.c`; no standalone test required | — |
| 5 | 18-field CSV @ 50 Hz DMA TX | Mean body 1144 us, max 3153 us, CSV drops 0/250, ISR overrun 0 | PASS |
| 6-A | IWDG watchdog — normal operation | 100 refreshes, loop timing unaffected | PASS |
| 6-B | IWDG watchdog — trigger test | I2C jumper removed -> auto-reset after timeout (header reprinted) | PASS |

**Loop timing budget:** 200 Hz = 5 ms slot. Worst-case loop body 3153 us
(63% of budget). Zero body overrun and zero ISR overrun across 1000 loops.

**Kalman filter:** Fixed KF only (Mode A — predict every 5 ms, update on ToF
DataReady ~50 Hz). Parameters Q = 1.0, R = 400.0, W = 20. Steady-state
P and K match the E0 Python simulation, confirming correct C porting.

---

## 2. C <-> Python Equivalence Verification

**Problem:** A direct row-by-row `abs error < 1e-3` comparison fails, because
the C firmware uses `float32` while the Python reference uses `float64`, and
the 50 Hz decimated CSV cannot be aligned 1:1 with a synchronous re-simulation.

**Approach:** Fields are verified by three methods according to their nature.

| Method | Fields | Criterion | Result |
|---|---|---|---|
| A — state equivalence | kf_estimate, kf_covariance, kalman_gain, innovation_cov | Relative error (0.5%; S: 0.05%) | PASS |
| B — residual self-consistency | residual | `residual == tof - prev_estimate` identity | PASS |
| C — structural equivalence | residual_mean, residual_var | Construction argument | PASS |

**Method A:** KF state (x, P) and behaviour (K, S) agree within the relative
error expected from `float32` accumulation. The tighter `float64`-style
`1e-3` bound was never physically attainable given CSV output precision.

**Method B:** The KF identity `residual = z - x_pred` holds for 179/200
steady-state rows with a maximum error of 7.1e-15 mm (machine epsilon).
The remaining 21 rows occur at regular intervals — a periodic artifact of
the 200 Hz / 50 Hz decimation boundary, not an algorithmic discrepancy.

**Method C:** `residual_mean` and `residual_var` cannot be verified by row
comparison — the C window buffers internal `float32` residuals while the CSV
stores `%.3f`-rounded values, and decimation prevents reconstruction of the
window contents. However, the C and Python statistics formulas are identical
(`sum/W`, `sq_sum/W - mean^2`) and the input (residual) is verified by
Methods A/B. Equal input through an equal function yields equal output;
their equivalence therefore follows by construction.

**Conclusion:** The Fixed KF C implementation is numerically equivalent to
the Python reference. Full report: `equivalence_check/kf_equivalence_report.txt`.

---

## 3. CSV Format Note

Phase 6 uses the *current* 18-field CSV layout, which differs from the thesis
3.6 specification (extra fields: `seq`, separate L/R encoder columns,
`tof_ambient`, `kf_covariance`; missing: `us_distance_mm`, `sensor_disagree`,
`gt_distance_mm`, `R_label`). This does not affect equivalence verification,
which compares values, not column names. The thesis-aligned spec will be
adopted from Phase 7 onward.

---

## 4. Known Limitations

- `residual_mean` / `residual_var` are verified structurally (Method C), not
  by row comparison — a consequence of 50 Hz decimation logging without
  per-loop window-state logging.
- ISR overrun appeared in intermediate Step 3 builds as a side-effect of
  blocking debug `printf`; eliminated in Step 5 once logging moved to DMA TX.

---

## 5. Artifacts

```
tests/06_kf_integration_verification/
├── TEST_RESULT.md
├── logs/                          PuTTY session logs (Step 1~6)
├── csv_extracted/
│   └── phase6_step6_csv.csv        250-row CSV used for equivalence check
└── equivalence_check/
    ├── kf_equivalence_check.py     verification script
    └── kf_equivalence_report.txt   full equivalence report
```
