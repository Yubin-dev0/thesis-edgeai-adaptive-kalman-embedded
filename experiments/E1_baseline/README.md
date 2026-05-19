# E1 — Baseline (Normal Wall Approach)

Baseline scenario for comparing the three algorithms (Fixed KF, CM-AKF,
TinyML-AKF) against the raw sensor reading. It establishes the basic
performance of each algorithm under a uniform white foam-board wall.
Corresponds to scenario E1 in thesis Table 4-4 (Section 4.2).

> **Status: measurement complete (2026-05-19).** Runs 1–5 all nominal.
> Procedure, results and analysis are in [`EXP_RESULT.md`](./EXP_RESULT.md).

## Demo Video

[![E1 measurement demo](https://img.youtube.com/vi/Y_JBkP_VsSM/maxresdefault.jpg)](https://youtu.be/Y_JBkP_VsSM)

> Click the thumbnail to open the video on YouTube.

## Folder Structure

```
experiments/E1_baseline/
├── README.md      this file — scenario intro and folder guide
├── EXP_RESULT.md  measurement procedure, results, analysis
│                  (parallel to TEST_RESULT.md under tests/)
├── logs/           logger_cli.py CSV logs (deliverable of the measurement role)
│   ├── E1_run01.csv
│   ├── E1_run02.csv
│   ├── E1_run03.csv
│   ├── E1_run04.csv
│   └── E1_run05.csv
└── analysis/      (optional) notes and plots for the post-measurement check
```

Post-processing outputs (e.g. processed/) are not kept here — they belong
to the post-processing role.

## Summary

- **Measurement method** — Fixed KF and CM-AKF run concurrently in a single
  firmware build and are logged to a 25-column wide CSV. TinyML-AKF is
  produced separately by PC-side post-hoc inference after model training.
- **Result** — runs 1–5 all nominal. Fixed-KF residual bias within ±1.3 mm,
  no CM-AKF R divergence, ToF status=0 ratio 100%.
- **Deliverable** — the five CSV logs under `logs/`. Subsequent
  post-processing (per-algorithm split, filename normalisation, GT column
  computation, TinyML post-hoc inference) is handled by the post-processing
  role.

See [`EXP_RESULT.md`](./EXP_RESULT.md) for details.
