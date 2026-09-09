# Edge-AI Adaptive Kalman Filter on STM32 — Real-Time Embedded Deployment

> Undergraduate thesis (HUFS, Dept. of Information & Communications Engineering, 2026)
> **"Edge AI 기반 적응형 칼만 필터의 임베디드 실시간 적용 연구"**
> *A Study on Real-Time Embedded Application of an Edge-AI-Based Adaptive Kalman Filter*

A bare-metal STM32F446RE robot that runs **three Kalman filters side by side at 200 Hz** — a fixed-R KF, a covariance-matching adaptive KF (CM-AKF, Mehra 1970), and a **TinyML-AKF whose measurement-noise covariance R̂ is predicted on-device by an INT8 MLP** — and logs them frame-aligned so the algorithms can be compared on identical sensor input.

The question was not "can a neural net beat a classical estimator" but **"can a learned R estimator run inside a hard real-time loop on a Cortex-M4, and does it behave differently from CM-AKF when the sensor misbehaves?"** Both answers are yes, and both are measured rather than claimed.

📄 Full thesis (Korean): [`docs/thesis_edge_ai_adaptive_kalman_2026.pdf`](docs/thesis_edge_ai_adaptive_kalman_2026.pdf)

---

## Key results (all from on-device measurement)

| Research question | Result |
|---|---|
| **RQ1 — Real-time feasibility** | TinyML inference **35.32 µs mean / 38.10 µs max / run-to-run std 0.007 µs** over **242,992 inferences** (3 × 30-min static runs). Main loop 1.24 ms mean, 3.58 ms max inside a 5 ms (200 Hz) period. **Zero overruns, zero watchdog resets.** |
| **RQ2 — R̂ recovery after sensor fault** | Under dynamic ToF occlusion (E3), TinyML R̂ recovers in **~60 ms vs ~160 ms for CM-AKF (2.7×)**. CM-AKF's R saturates at its 10,000 mm² clamp while the sensor is stuck — a structural limit of residual-only adaptation, reproduced on real hardware. |
| **RQ3 — Multivariate features** | ToF `signal_rate` (F6) changes **~80 ms before the residual does** — a leading indicator that a residual-only method cannot see. Transparent acrylic returns *higher* signal rate (13.2 Mcps) than black foam board (11.1 Mcps), a counter-intuitive result that justifies feeding raw optical quality into the estimator. |
| **Position RMSE (honest framing)** | TinyML-AKF does **not** beat CM-AKF on position RMSE. Both adaptive filters clearly beat the fixed KF (E3: 14.2 / 16.6 mm vs 44.9 mm; E2 white: 5.5 / 6.8 mm vs 9.7 mm; E5 unseen surface: 5.1 / 5.6 mm vs 6.2 mm). The contribution is the **embedded deployment validation and R̂ dynamics**, not a new filter. |
| **Generalization (E5)** | On a grey surface excluded from training, TinyML R̂ stays bounded ([1.7, 321] mm², no clamp saturation) and lands in the same region as CM-AKF's R̂. |

---

## System overview

```
                  ┌──────────────────────── STM32F446RE @ 180 MHz ───────────────────────┐
 VL53L0X ToF ─I2C─┤                                                                      │
 (range, status,  │   200 Hz TIM6 ISR-paced loop                                         │
  signal_rate)    │   ┌───────────┐   ┌────────────┐   ┌───────────────────────────┐    │
                  │   │ Fixed KF  │   │  CM-AKF    │   │ TinyML-AKF                │    │
 2× quadrature ───┤──▶│ R = R₀    │   │ R̂ = f(res) │   │ 6 features ─▶ INT8 MLP ─▶ R̂ │    │
 encoders (TIM2/4)│   └─────┬─────┘   └─────┬──────┘   └────────────┬──────────────┘    │
                  │         └───────────────┴─────────────┬─────────┘                   │
 HC-SR04 ─TIM3────┤                                        ▼                             │
                  │             28-column CSV frame  ──USART6 + DMA──▶ HC-06 BT ──▶ PC   │
 TB6612FNG motors ┤                                                                      │
 (TIM1 PWM)       └──────────────────────────────────────────────────────────────────────┘
```

- **Plant model**: 1-D scalar position KF (distance to wall), encoder odometry as prediction, ToF as measurement.
- **CM-AKF**: Mehra (1970) covariance matching, implemented at full strength — no hand-weakening of the baseline to make the learned model look better.
- **TinyML-AKF**: Keras MLP `6 → 16 → 8 → 1`, INT8-quantized, **257 parameters, ~3.2 KB**. Trained on CM-AKF's `log1p(R)` as a pseudo-label; output `expm1`'d and clamped to `[1, 10000]` mm². Six input features (F1–F6): residual statistics, encoder/ToF disagreement, ToF measurement-validity rate, and ToF `signal_rate`.
- **Deployment**: C code generated with **ST Edge AI Core 4.0 (`stedgeai` CLI)**, integrated into the STM32CubeIDE project by hand using the `STAI_FLAG_PREALLOCATED` I/O pattern. Inference runs in the same 200 Hz loop as the filters.
- **Instrumentation**: DWT cycle counter for per-inference latency, IWDG watchdog (~2 s), sequence numbers on every frame for wireless-drop reconstruction.

---

## Experiments

Every scenario logs Fixed / CM / TinyML estimates in the same row, so comparisons are on identical input.

| ID | Scenario | Purpose | Folder |
|---|---|---|---|
| E0 | Synthetic 1-D simulation | Validate KF / CM-AKF implementation, choose P₀ = R₀ | `simulation/` |
| R0 | 500 mm static calibration | Fix R₀ = 24 mm² (σ ≈ 4.87 mm, 232 samples) | `experiments/R0_calibration/` |
| E1 | Baseline approach, white board | Reference behaviour, normalization statistics | `experiments/E1_baseline/` |
| E2 | Reflectivity: white / black / acrylic | Surface-dependent noise, training set | `experiments/E2_Reflectivity/` |
| E3 | Dynamic aluminium-panel occlusion | Stuck-sensor fault, R̂ recovery (RQ2) | `experiments/E3_dynamic_occlusion/` |
| E4 | 3 × 30 min static | Latency determinism, drift, watchdog (RQ1) | `experiments/E4_static_stability/` |
| E5 | Grey board (unseen surface) | Out-of-distribution generalization | `experiments/E5_unknown_surface/` |

Each folder contains an `EXP_RESULT.md` (conditions, firmware version, checklist, findings) and the raw CSV logs under `logs_final/`.

**Design notes worth knowing before reading the data**
- E1–E3 and E5 runs are **manually rolled** (105–134 mm/s), not motor-driven; the filter is speed-independent and this is documented per run.
- E3 was redesigned mid-project: the planned black-foam occluder never produced `range_status ≠ 0`, so a hand-inserted aluminium panel was used instead.
- The CSV grew from 18 → 25 → 28 columns across firmware stages; `tools/logger_cli.py` parses the `# CSV_HEADER:` line so it works with all of them.
- F5 (`tof_meas_rate`) is 1.000 for all E1/E2 rows — only E3 exercises it. Recorded as a data limitation, not patched after the fact.

---

## Hardware verification (Phase 0–7)

Before any filter ran, every subsystem was verified in isolation with a written pass/fail report:

| Phase | Subsystem | Report |
|---|---|---|
| 0 | Power rail / battery ADC | `tests/00_power_verification/` |
| 1 | VL53L0X ToF (I2C1) — noise vs distance, R₀ at 100 mm | `tests/01_vl53l0x_verification/` |
| 2 | Quadrature encoders (TIM2/TIM4, 3840 PPR, 66 mm wheel) | `tests/02_encoder_verification/` |
| 3 | HC-SR04 (TIM3 input capture) | `tests/03_hcsr04_verification/` |
| 4A / 4B | Motor standalone / motor-induced sensor noise | `tests/04A_…/`, `tests/04B_…/` |
| 5 | HC-06 Bluetooth (USART6 + TX DMA, 115200) | `tests/05_hc06_bluetooth_verification/` |
| 6 | KF integration + **bit-level equivalence** of MCU KF vs reference C/Python KF | `tests/06_kf_integration_verification/` |
| 7 | Full manual integration | `tests/07_manual_integration_verification/` |

Phase 6 is the one I'd point a reviewer to: the on-target Kalman filter was checked against a host-side reference implementation on the same input stream (`equivalence_check/`), so later RMSE differences are attributable to the R-estimation strategy and not to arithmetic drift.

---

## Repository layout

```
firmware/          STM32CubeIDE project — Core/Src/main.c, kalman_filter.c/h,
                   stedgeai-generated network.c / network_data.c, Middlewares/ST/AI runtime, firmware.ioc
simulation/        E0 — 1-D KF / CM-AKF Python simulation, synthetic data, metric scripts
experiments/       R0 calibration + E1–E5 measurement logs and EXP_RESULT.md reports
tests/             Phase 0–7 hardware verification logs and TEST_RESULT.md reports
tools/
  logger_cli.py         Serial CSV logger (auto-parses CSV_HEADER, any column count)
  tinyml/               Training pipeline, 6-feature main model, 3-feature ablation, .tflite/.keras
  verification/         Host-side reference KF (kf_verify.c) and CSV generator for Phase 6
docs/              Thesis PDF
```

---

## Hardware

| Item | Detail |
|---|---|
| MCU | STM32F446RE NUCLEO, 180 MHz (HSE bypass), FPU |
| Distance | VL53L0X ToF on I2C1 (PB8/PB9); HC-SR04 on TIM3 CH1 (PA6) / trigger PA1 |
| Odometry | 2× quadrature encoders, TIM2 (PA15/PB3) & TIM4 (PB6/PB7), 4× decoding |
| Drive | TB6612FNG on PC8–PC12, PWM on TIM1 CH1/CH2 (PA8/PA9) |
| Telemetry | HC-06 Bluetooth, USART6 (PC6/PC7) with TX DMA |
| Timing | TIM6 200 Hz main loop, DWT cycle counter, IWDG ~2 s |
| Power monitor | ADC1 IN4 (PA4) |

---

## Reproducing

**Firmware** — open `firmware/` in STM32CubeIDE (`firmware.ioc` holds all pin assignments), build, flash to the NUCLEO. Scenario and run length are compile-time constants (`CSV_SCENARIO_ID`, `PHASE6_N_TEST_LOOPS`).

**Logging**
```bash
pip install pyserial
python tools/logger_cli.py --port COM5 --out experiments/E1_baseline/logs/run01.csv
```

**Simulation / metrics**
```bash
pip install numpy pandas matplotlib
python simulation/kf_simulation_1D.py
python simulation/kf_eval_metrics.py experiments/E5_unknown_surface/logs_final/
```

**TinyML retraining**
```bash
pip install tensorflow scikit-learn
python tools/tinyml/tinyml_train.py        # 6-feature main model → tools/tinyml/models/
python tools/tinyml/train_ablation.py      # 3-feature ablation
stedgeai generate --model tools/tinyml/models/6feat_main.tflite --target stm32f4   # ST Edge AI Core 4.0
```

---

## Limitations & what I'd do next

- The plant is a **1-D scalar model**; the interesting failure modes of a full 2-D pose filter (heading drift, slip) are out of scope.
- Disturbances in E3 last ~1 s, which is too short for the faster R̂ recovery to accumulate into a position-RMSE advantage. A longer or repeated-occlusion scenario is the obvious follow-up.
- The TinyML label is CM-AKF's own R, so the model can only be as good as the teacher on in-distribution data; its added value shows up in *timing* and *robustness*, not steady-state accuracy.
- Motor-driven runs at controlled speed would remove the operator as a variable.

---

## Contributors

| | Role |
|---|---|
| **Yubin Shin** ([@Yubin-dev0](https://github.com/Yubin-dev0)) | Hardware bring-up and Phase 0–7 verification, all STM32 firmware (C), KF / CM-AKF / TinyML-AKF implementation, ST Edge AI Core deployment, experiment design and measurement (E1–E5), thesis Ch. 1, 3, 4.1–4.2, 4.4, 5 |
| **Dayoung Lim** | Python analysis tooling and data dashboard, metric post-processing, thesis Ch. 2 and Ch. 4 formatting, presentation materials |

Advisor: Prof. Kyeong-Rak Son — HUFS Dept. of Information & Communications Engineering.

---

## Reference

R. K. Mehra, "On the identification of variances and adaptive Kalman filtering," *IEEE Trans. Automatic Control*, vol. 15, no. 2, pp. 175–184, 1970.
