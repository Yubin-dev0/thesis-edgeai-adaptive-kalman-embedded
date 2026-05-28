# E4 — Static Long-Term Stability (30-min Stationary Logging)

Long-term stability scenario for verifying the embedded system's real-time
behaviour over an extended run. The robot is held **stationary (motors OFF)**
at ~500 mm from a white foam-board wall and logged continuously for 30
minutes, three times. The goal is to isolate long-term factors — battery
discharge, thermal effects, R̂ drift, cumulative inference latency, memory
stability — from any measurement-noise change, by holding the surface
variable constant (same white foam-board as E1).
Corresponds to scenario E4 in thesis Table 4-4 (Section 4.2).

> **Status: measurement complete (2026-05-20).** 3 runs × 30 min nominal.
> All checklist items met: 30-min runs completed with 0 overruns, main-loop
> mean 1.24 ms, CM-AKF R̂ run-to-run CV 0.44%, TinyML inference 35.00 µs
> (std 0.007 µs) over 242,085 inferences. Measured with the stage 4-C
> firmware (on-device TinyML, 28-column CSV, `PHASE6_N_TEST_LOOPS=360000`).
> Procedure, results and analysis are in [`EXP_RESULT.md`](./EXP_RESULT.md).

## Folder Structure

```
experiments/E4_StaticStability/
├── README.md      this file — scenario intro and folder guide
├── EXP_RESULT.md  measurement procedure, results, analysis
│                  (parallel to TEST_RESULT.md under tests/)
├── logs_final/    final 4-C firmware logs (deliverable of the measurement role)
│   ├── E4_run01.csv
│   ├── E4_run02.csv   (header manually restored — see EXP_RESULT §8)
│   └── E4_run03.csv
└── analysis/      (optional) notes and plots for the post-measurement check
```

Post-processing outputs (e.g. processed/) are not kept here — they belong
to the post-processing role.

## Summary

- **Measurement method** — Fixed KF, CM-AKF, and TinyML-AKF run concurrently
  in a single stage 4-C firmware build (same as E2/E3) and are logged to a
  28-column wide CSV. The only changes for E4 are the run length
  (360,000 loops = 30 min) and two long-run instrumentation fixes:
  `tinyml_infer_us` switched to per-row last value, and `ai_infer_cycles_sum`
  widened to `uint64_t` to prevent overflow at ~24 min.
- **Setup** — robot **stationary, motors OFF**, white foam-board wall at
  500 mm (E1-consistent surface, deliberately held constant), MDF floor.
- **Result** — 3 runs × 30 min nominal. Main-loop body time 1239 µs mean
  (27.5% of the 4500 µs budget), 0 overruns over 1,080,000 loops. CM-AKF R̂
  converges to ≈ 22.9 (≈ KF_R_INIT 24) with no 30-min drift (5-min segment
  deviation ±1.61% max). Run-to-run R̂ CV 0.44%. TinyML inference
  deterministic at 35.00 µs (std 0.007 µs, max 38 µs), 7% of the 500 µs
  budget.
- **Key finding** — the system is timing-deterministic: main-loop and
  inference times reproduce to µs resolution across all 3 runs. This fully
  answers RQ1 and supplies the complete timing dataset for thesis 4.3, so
  **no separate TinyML-only timing firmware is needed**. In the static
  condition CM-AKF behaves like Fixed KF (adaptation inactive) and TinyML
  reproduces CM-AKF in-distribution — confirming the adaptive advantage must
  appear in the dynamic scenarios (E2/E3/E5).
- **Note — no demo video.** Unlike the rolling scenarios (E1/E2/E3), E4 is a
  30-min stationary run with no visible motion, so no demo video is provided.
- **Deliverable** — the three CSV logs under `logs_final/`. Subsequent
  post-processing (per-algorithm split, filename normalisation, GT column
  computation) is handled by the post-processing role.

See [`EXP_RESULT.md`](./EXP_RESULT.md) for details.
