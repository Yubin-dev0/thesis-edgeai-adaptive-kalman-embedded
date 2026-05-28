# E2 — Reflectivity (Wall Surface Reflectivity Variation)

Reflectivity scenario for comparing the three algorithms (Fixed KF, CM-AKF,
TinyML-AKF) against the raw sensor reading under varying wall reflectivity.
Three wall materials (white foam-board, black foam-board, transparent
acrylic) produce a monotonically separated measurement-noise environment.
Corresponds to scenario E2 in thesis Table 4-4 (Section 4.2).

> **Status: learning-set measurement complete (2026-05-20).** 9 runs
> (white ×3, black ×3, acrylic ×3) nominal. Measured with the stage 4-C
> firmware (on-device TinyML, 28-column CSV). Procedure, results and
> analysis are in [`EXP_RESULT.md`](./EXP_RESULT.md).

## Demo Video

[![E2 measurement demo](https://img.youtube.com/vi/AOoWN997JHg/maxresdefault.jpg)](https://youtu.be/AOoWN997JHg)

> Click the thumbnail to open the video on YouTube.

## Folder Structure

```
experiments/E2_Reflectivity/
├── README.md      this file — scenario intro and folder guide
├── EXP_RESULT.md  measurement procedure, results, analysis
│                  (parallel to TEST_RESULT.md under tests/)
├── logs/          (reference) 25-column logs from the 4-B-2 firmware
│                  (TinyML disabled), measured earlier the same day
├── logs_final/    final 4-C firmware logs (deliverable of the measurement role)
│   ├── E2_white_run01.csv
│   ├── E2_white_run02.csv
│   ├── E2_white_run03.csv
│   ├── E2_black_run01.csv
│   ├── E2_black_run02.csv
│   ├── E2_black_run03.csv
│   ├── E2_acryl_run01.csv
│   ├── E2_acryl_run02.csv
│   └── E2_acryl_run03.csv
└── analysis/      (optional) notes and plots for the post-measurement check
```

Post-processing outputs (e.g. processed/) are not kept here — they belong
to the post-processing role.

## Summary

- **Measurement method** — Fixed KF, CM-AKF, and TinyML-AKF run concurrently
  in a single stage 4-C firmware build and are logged to a 28-column wide
  CSV. Unlike E1, the TinyML-AKF estimate is produced **on the MCU in real
  time** (X-CUBE-AI runtime), not by PC post-hoc inference.
- **Materials** — white foam-board (baseline), black foam-board (low
  reflectivity), transparent acrylic B4 3 mm (extreme, measured with a
  5–10° tilt to the ToF axis).
- **Result** — 9 learning runs nominal. signal_rate monotonically separated
  (white 20.5 → black 10.1 → acrylic 13.2 MCps); CM-AKF R monotonic on
  stable runs (white 32.7 → black 65.2 → acrylic 127.3 mm²); TinyML R
  follows at ~50–80% of CM-AKF R. On-device inference within the 200 Hz
  budget.
- **Key finding** — transparent acrylic did not reproduce `range_status≠0`
  (940 nm IR partially transmits through acrylic), so F5 stays constant in
  E2 and its learning data comes solely from E3. Thesis 4.2 body updated
  accordingly.
- **Deliverable** — the nine CSV logs under `logs_final/`. Subsequent
  post-processing (per-algorithm split, filename normalisation, GT column
  computation) is handled by the post-processing role.

See [`EXP_RESULT.md`](./EXP_RESULT.md) for details.
