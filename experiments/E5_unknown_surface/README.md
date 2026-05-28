# E5 — Unknown Surface Generalization (Untrained Wall Reflectivity)

Generalization scenario testing whether the on-device TinyML-AKF, trained on
the E2 material set (white foam-board, black foam-board, transparent acrylic),
extends to a wall reflectivity **absent from the training distribution**. A
single mid-reflectivity material — grey single-sided foam-board 8-section
5 T — is rolled toward, and the three algorithms (Fixed KF, CM-AKF,
TinyML-AKF) are compared against the raw sensor reading.
Corresponds to scenario E5 in thesis Table 4-4 (Section 4.2).

> **Status: measurement complete (2026-05-21).** 5 runs nominal. Measured
> with the stage 4-C firmware (on-device TinyML, 28-column CSV). TinyML-AKF
> produced bounded R estimates on the untrained surface (no clamp saturation,
> no divergence). Procedure, results and analysis are in
> [`EXP_RESULT.md`](./EXP_RESULT.md).

## Demo Video

[![E5 measurement demo](https://img.youtube.com/vi/5Yfbs8R6QQc/maxresdefault.jpg)](https://youtu.be/5Yfbs8R6QQc)

> Click the thumbnail to open the video on YouTube.

## Folder Structure

```
experiments/E5_unknown_surface/
├── README.md      this file — scenario intro and folder guide
├── EXP_RESULT.md  measurement procedure, results, analysis
│                  (parallel to TEST_RESULT.md under tests/)
├── logs_final/    final 4-C firmware logs (deliverable of the measurement role)
│   ├── E5_run01_20260521_002220.csv
│   ├── E5_run02_20260521_002319.csv
│   ├── E5_run03_20260521_002355.csv
│   ├── E5_run04_20260521_002431.csv
│   └── E5_run05_20260521_002537.csv
└── analysis/      (optional) notes and plots for the post-measurement check
```

Post-processing outputs (e.g. processed/) are not kept here — they belong
to the post-processing role.

## Summary

- **Measurement method** — Fixed KF, CM-AKF, and TinyML-AKF run concurrently
  in a single stage 4-C firmware build and are logged to a 28-column wide
  CSV. As in E2, the TinyML-AKF estimate is produced **on the MCU in real
  time** (X-CUBE-AI runtime), not by PC post-hoc inference. Same firmware as
  E2/E3/E4 except the per-scenario `PHASE6_N_TEST_LOOPS` (1000, ~5 s/run).
- **Material** — grey single-sided foam-board 8-section 5 T, deliberately
  **not** part of the TinyML training set (white/black foam-board +
  transparent acrylic). It probes a reflectivity region the model never saw.
- **Result** — 5 runs nominal, status=0 at 100% all runs. On stable runs
  (run01–04) CM-AKF R mean ≈ 18 and TinyML R mean ≈ 17 (TinyML ≈ 90–95% of
  CM-AKF on grey, higher than the ~50–80% seen on the trained E2 materials).
  TinyML R stayed bounded within [1.74, 321.26] across all 5 runs and never
  hit the clamp limits (1.0 / 10000.0) — the model neither saturated nor
  diverged on the untrained surface, the core RQ2/RQ3 generalization evidence.
- **Key finding** — grey foam-board did not occupy the "mid-reflectivity"
  position the thesis hypothesis assumed. Its signal_rate (mean ≈ 16 MCps)
  sits between black (≈ 10) and white (≈ 20) but closer to white, and its
  CM-AKF R (≈ 18 on stable runs) is **lower** than every trained E2 material.
  Grey turned out to be an *easier* noise environment than expected, not a
  middle one. status≠0 again did not occur, so F5 stays constant in E5 (as
  in E1/E2); F5 learning data remains sourced solely from E3.
- **Deliverable** — the five CSV logs under `logs_final/`. Subsequent
  post-processing (per-algorithm split, filename normalisation, GT column
  computation) is handled by the post-processing role.

See [`EXP_RESULT.md`](./EXP_RESULT.md) for details.
