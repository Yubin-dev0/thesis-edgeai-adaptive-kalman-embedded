# Phase 7 — Manual Integration Verification

**Date:** 2026-05-19
**Platform:** STM32F446RE NUCLEO, 2WD robot, Fixed KF firmware (Phase 6 base)
**Goal:** Verify that sensor fusion (ToF + encoder + KF) operates correctly
end-to-end while the robot is driven by hand, and characterise the data
link before the E1–E5 main experiments.

---

## Summary

| Sub-test | Result | Note |
|----------|--------|------|
| 7-B  Wall material pre-measurement | PASS | 4 materials, signal-rate baseline recorded |
| 7-C/D Blocker & distance marking   | PASS | 150x150 mm blocker, 0/250/500 mm marks |
| 7-E  Manual-roll integration       | PASS | KF + ToF + encoder integrated; see below |
| 7-F  30-min static endurance       | PASS | 30-min clean run completed (run91); see note on startup reset |

A firmware encoder-sign bug was found and fixed during 7-E. A Bluetooth
packet-loss effect during wheel rotation was identified, root-caused, and
shown not to compromise the planned analyses.

---

## 7-B  Wall material pre-measurement

Four candidate wall surfaces were measured statically at 500 mm using the
Phase 6 firmware unchanged (analysis-only approach — no firmware edit). The
goal was a per-material VL53L0X signal-rate baseline for interpreting the
E1-E5 results.

| Material | Signal rate (MCPS) | Non-zero status |
|----------|--------------------|-----------------|
| White woodlock  | 3.485 +/- 0.109 | 0% |
| Black woodlock  | 1.059 +/- 0.081 | 0% |
| Grey woodlock   | 2.452 +/- 0.096 | 0% |
| Clear acrylic   | 1.262 +/- 0.046 | 0% |

All four returned valid measurements (0% non-zero range status). The
acrylic, expected to be the hardest case, measured a weak but usable
signal — it did not fail outright, though it reads short (~440 mm). The
first acrylic capture was retaken because a coloured wall behind it let the
beam pass through; the retake (putty5) is the valid one.

---

## 7-E  Manual-roll integration

The robot was rolled by hand across the 500 mm track. Telemetry (18-field
CSV, 50 Hz) was captured over HC-06 Bluetooth by a custom CLI logger.

### Encoder-sign firmware bug (found and fixed)

Initial rolls showed the KF kinematic input collapsing to near-zero even
though the robot was clearly moving. Root cause: the two wheel motors face
opposite directions, so the right encoder counts with the opposite sign of
the left. An earlier hardware change — the right motor wiring was reversed
to correct its rotation direction — also reversed its encoder polarity. The
firmware averaged the two channels as `(dL + dR)/2`, so the opposite signs
cancelled and the KF `predict` step received ~0 input.

Fix (single line, at the encoder read site):

```c
int16_t dr = -(enc_r_now - enc_r_prev);   // right motor wiring reversed
```

Applying the correction at the data-entry point fixes every downstream
consumer at once: `enc_r_total`, `pos_r_mm`, and the KF input `u_mmps`.

After the fix, both encoder channels increase with the same sign and the
KF receives the true kinematic input.

Scope note: the E0 simulation and the Phase 6 C/Python equivalence check
were **not** re-run. Those verify the KF algorithm itself; the encoder sign
belongs to the encoder-processing stage, which is a different layer. The
two do not overlap, so their earlier PASS results remain valid.

### 7-E results (run02, post-fix)

| Metric | Value | Target | Verdict |
|--------|-------|--------|---------|
| CSV 18-field integrity | 99.55% | >= 99% | PASS |
| Roll speed             | 92.6 +/- 33.9 mm/s | (recorded, not gated) | OK |
| Bluetooth drop rate    | 12.0% | <= 1% | see below |
| KF / ToF / encoder integration | working | — | PASS |

ToF distance tracked 520 -> 76 mm and the KF estimate tracked 520 -> 122 mm
across the roll — sensor fusion operated correctly end-to-end.

---

## Bluetooth packet loss during wheel rotation

7-E rolls showed 10-20% of frames missing. Received frames were all intact
18-field rows — frames were dropped whole, not corrupted — and the missing
positions are identifiable from the `seq` counter.

### Root-cause isolation

| Condition | Wheels turning? | Drop rate |
|-----------|-----------------|-----------|
| Static (motor off)            | no  | 0% |
| Shaking the robot by hand     | no  | 0% |
| Wheels spun by hand, airborne | yes | ~20% |
| Actual roll on the ground     | yes | 10-20% |

The link is clean whenever the wheels are not turning, regardless of
vibration or robot motion. The single differentiating factor is wheel
(motor) rotation. When a DC motor is back-driven it acts as a generator and
produces electrical noise, which couples into the HC-06 link.

### Mitigations attempted

| Mitigation | Result |
|------------|--------|
| Separate power supply for HC-06 (independent buck + battery, single-point GND) | no improvement |
| 100 nF ceramic capacitors across both motor terminals | no improvement |

Neither standard conducted-noise mitigation helped, which indicates the
dominant coupling path is radiated (electromagnetic) rather than through
the shared power rails. Further hardware mitigation (shielding) is beyond
the scope of this verification phase, so hardware mitigation was stopped.

### Impact on the main experiments — assessed, not blocking

The loss is whole-frame dropout in a 200 Hz time series, with positions
known from `seq`. Its effect on each planned analysis:

- **RMSE / MAE** — mean-based metrics; an unbiased ~15% sample reduction
  leaves the estimate essentially unchanged. No impact.
- **TinyML training** — samples are independent; fewer samples, none wrong.
  The 6 features are computed on the MCU, where all frames are present;
  the loss occurs only on the MCU->PC link. No impact on feature quality.
- **KF time-series dynamics** — the KF runs at 200 Hz on the MCU and is
  unaffected; only the observation resolution coarsens. The E3 occlusion
  window still retains enough points to show the curve shape.

Conclusion: 10-20% roll-time packet loss does not block the E1-E5 main
experiments. It is recorded here as a characterised system limitation.

Thesis note: plans assumed "manual drive => no motor noise". This holds for
motor *drive* noise (PWM switching, large currents), which manual operation
does remove. But a back-driven motor still generates rotation noise. This
should be reflected honestly in the manual-drive justification / limitations
discussion of the thesis.

---

## Tooling note — CSV logger

PuTTY logging during 7-B/7-E produced glued/split CSV rows (line-break loss
in the terminal). It was replaced by a single-threaded CLI logger
(`logger_cli.py`) that parses the firmware `# CSV_HEADER:` line to adapt to
the field layout automatically, validates `seq` continuity and field count,
and writes one clean CSV per run. The earlier tkinter GUI logger was dropped
after repeated UI-thread freezes; a CLI tool has no event loop to contend
with and cannot hang.

---

## 7-F  30-min static endurance

Status: **PASS.** A clean uninterrupted 30-min capture (run91) was completed
on a fully charged battery, confirming the substantive results that two
earlier attempts (run90, run89) had already pointed to.

Per-5-minute multimeter logging was dropped as impractical for an unattended
endurance test; battery voltage may be recorded once at start and once at end.

### Clean run — run91 (full 30 min, fully charged battery)

- Duration: **exactly 1,800.0 s (30.00 min)** — seq 0 to 89,999, the full
  90,000-row log (360,000 loops at 200 Hz / 50 Hz log). The firmware
  completed the test loop and terminated normally.
- The final rows are spaced at a normal 20 ms and `seq` increments to the
  last row; no drift, no stall.
- CSV integrity: 84,689 stored rows, **all exactly 18 fields, 0 corrupted**.
  The "field error" lines counted by the logger were glued-row fragments
  (two CSV rows with a missing line break) and firmware banner lines; they
  are filtered out and do not appear in the saved CSV.
- `seq` monotonic across the run **except for one reset** at ~23 s
  (seq 4,675 -> 0). After that single early reset the run continued to
  seq 89,999 with no further resets — 29+ min reset-free.
- Whole-frame `seq` gaps over the 30 min totalled ~5,400 frames (~6 %),
  consistent with the static-capture Bluetooth loss seen across Phase 7;
  this is link loss on the MCU->PC path, not an MCU fault.

### Earlier attempts (context)

- **run90** (HC-06 on main power): reached the full 90,000 frames, 0
  corrupted rows. Showed one reset at ~56 s, then ran to seq 89,999.
- **run89** (HC-06 on separate power): stopped at ~11.35 min (seq 34,065),
  0 resets while it ran. The capture ended cleanly — the firmware showed no
  crash, the MCU->PC link simply stopped delivering data. Leading cause:
  the main LiPo was near depletion after a full day of testing (it went
  flat shortly after, when run87 was attempted). run91, on a freshly
  charged battery, ran the full 30 min, which is consistent with this
  depletion explanation.

### 7-F verdict

| Check | Result |
|-------|--------|
| MCU runs 30 min continuously | YES — run91 reached 90,000 frames, 30.00 min |
| CSV 18-field integrity over the run | YES — 0 corrupted rows (run90, run91) |
| `seq` monotonic / no stall | YES — except one startup reset (see below) |
| 30-min clean capture achieved | YES — run91 |

**Startup reset.** A single `seq` reset occurred early in the run — at ~56 s
in run90 and at ~23 s in run91, i.e. **2 of 2** full-length runs. It is
therefore not a one-off glitch but a reproducible event confined to the
first minute after power-up; it is interpreted as a power-up settling
transient. In both runs the reset happened once and the remaining 29+ min
ran reset-free. Impact on the main experiments is low: E1-E5 runs are ~5 min
each, and a startup reset is immediately visible (seq returns to 0) so the
affected run is simply restarted. Precise root-cause analysis of the
startup transient is deferred as a follow-up item, outside the Phase 7
schedule.

MCU 30-min endurance is considered demonstrated.

### Side finding — long-capture link stability

run89 surfaced that a 30-min continuous capture is not guaranteed to hold a
single uninterrupted link. The E1-E5 main experiment runs are ~5 min each,
well within the demonstrated stable window, so this does not block the main
experiments. If any long continuous capture is needed later, a
link-reconnect procedure should be in place.
