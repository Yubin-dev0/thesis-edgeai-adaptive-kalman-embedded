# E3 — Dynamic Occlusion (Transient ToF Beam Interference)

Occlusion scenario for comparing the three algorithms (Fixed KF, CM-AKF,
TinyML-AKF) against the raw sensor reading when the ToF beam is transiently
interfered with. An A4 aluminium panel is dynamically inserted into the beam
path for ~0.75–1 s, producing a persistent measurement-jump outlier that
reproduces the CM-AKF stuck-sensor limit (thesis 2.2.3). Corresponds to
scenario E3 in thesis Table 4-4 (Section 4.2).

> **Status: learning + evaluation measurement complete (2026-05-20).**
> 5 learning runs (4-B-2 firmware, Fixed/CM only) + 5 evaluation runs (stage
> 4-C firmware, on-device TinyML, 28-column CSV). Occlusion reproduced in
> every run; CM-AKF R reaches the 10000 clamp (stuck-sensor limit confirmed).
> Procedure, results and analysis are in [`EXP_RESULT.md`](./EXP_RESULT.md).

## Demo Video

[![E3 measurement demo](https://img.youtube.com/vi/qZ95YBaswlU/maxresdefault.jpg)](https://youtu.be/qZ95YBaswlU?si=5GyKnWWXASHAbPTb)

> Click the thumbnail to open the video on YouTube.

## Folder Structure

```
experiments/E3_Dynamic_Occlusion/
├── README.md      this file — scenario intro and folder guide
├── EXP_RESULT.md  measurement procedure, results, analysis
│                  (parallel to TEST_RESULT.md under tests/)
├── logs/          learning-set logs from the 4-B-2 firmware
│                  (TinyML disabled, Fixed/CM only) — run01–05
├── logs_final/    evaluation logs from the final 4-C firmware
│                  (on-device TinyML, 28 columns) — run01–05
│   ├── E3_run01.csv
│   ├── E3_run02.csv
│   ├── E3_run03.csv
│   ├── E3_run04.csv
│   └── E3_run05.csv
└── analysis/      (optional) notes and plots for the post-measurement check
```

Post-processing outputs (e.g. processed/) are not kept here — they belong
to the post-processing role.

## Summary

- **Measurement method** — Fixed KF, CM-AKF, and TinyML-AKF run concurrently
  in a single stage 4-C firmware build and are logged to a 28-column wide
  CSV. The TinyML-AKF estimate is produced **on the MCU in real time**
  (X-CUBE-AI runtime).
- **Mechanism** — the robot rolls one-way toward a white wall (500 mm start);
  when it passes the 250 mm tape mark, the operator inserts an A4 aluminium
  panel into the ToF beam at the 100 mm point for ~0.75–1 s. The strong
  940 nm IR reflection makes the measured distance jump from the true wall
  distance (~250 mm) to the panel distance (~100–200 mm), a persistent
  outlier with `status` staying 0.
- **Result** — occlusion reproduced in all 10 runs; CM-AKF R reaches the
  10000 clamp in 9/10 runs, with delayed recovery after withdrawal —
  the empirical stuck-sensor limit (RQ2). TinyML R clamps during occlusion
  and decays gradually after, distinct from Fixed (R = 24 constant).
  On-device inference within the 200 Hz budget (RQ1).
- **Key finding — scenario redesign** — the original static black-foam-board
  side-blockage was found, across 7 pre-trials, **not** to induce
  `range_status≠0`: black foam is a visible-light absorber but only a partial
  940 nm IR absorber, so the ToF reads it as a reflective surface rather than
  failing. The scenario was redesigned to dynamic aluminium occlusion
  (measurement-jump outlier), which is functionally equivalent for the
  RQ2/RQ3 challenge. Thesis 4.2 body and Table 4-4 updated accordingly.
- **F5** — with `range_status≠0` at 0 across E1/E2/E3, F5 (Measurement Rate)
  stays constant 1.000; it is retained as a reserve channel and documented as
  a data limitation.
- **Deliverable** — the five evaluation CSV logs under `logs_final/` and the
  five learning logs under `logs/`. Learning and evaluation sets use distinct
  firmware versions and must not be mixed (data-leakage guard). Subsequent
  post-processing (per-algorithm split, filename normalisation, GT column
  computation) is handled by the post-processing role.

See [`EXP_RESULT.md`](./EXP_RESULT.md) for details.
