# Phase 0 — Power Supply Verification

## Objective

Verify the integrity of the Buck Converter (YwRobot PWR060010, LM2596, fixed 5V) and LiPo 7.4V 2S battery as the primary power source for sensor and logic circuits. This phase establishes a stable, low-noise 5V rail for sensor circuits and confirms that previously verified sensor behavior (Phase 3 HC-SR04) is preserved when the supply is migrated from the temporary NUCLEO USB 5V to the Buck 5V output.

## Hardware Under Test

| Item | Specification |
|---|---|
| Battery | LiPo 7.4V 2S |
| Buck Converter | YwRobot PWR060010 (LM2596, fixed 5V output, screw terminal) |
| Output decoupling | 100μF electrolytic capacitor (Buck output) |
| Multimeter | DC 20V range |
| Verification load (0-6) | HC-SR04 ultrasonic module (Phase 3 firmware) |

## Test Procedure and Results

### 0-1 LiPo Cell Voltage Measurement

Per-cell voltage measured via the LiPo balance connector (3-pin) using a multimeter on DC 20V range. Cell 1 measured between GND and the middle pin, Cell 2 between the middle pin and the end pin, total between GND and the end pin.

**Pass criteria:** each cell ≥ 3.7V, total ≥ 7.4V, inter-cell deviation ≤ 0.05V.
**Result:** Pass. Detailed per-cell voltage values recorded in the Notion test log.

### 0-2 Buck Converter No-Load Output

LiPo main connector wired to Buck IN+/IN− with polarity verified prior to connection. Output voltage measured at OUT+/OUT− with no load attached. The YwRobot PWR060010 is fixed-output and does not require trimmer adjustment.

**Pass criteria:** output voltage 5.00V ± 0.1V (4.9 ~ 5.1V).
**Result:** Pass. Detailed numerical value recorded in the Notion test log.

### 0-3 Buck Converter Load Stability

Multimeter kept connected to the Buck output for approximately one minute to observe voltage stability under a minimal load.

**Pass criteria:** output voltage maintained within 4.9 ~ 5.1V with no observable drop or fluctuation.
**Result:** Pass. Detailed numerical value recorded in the Notion test log.

### 0-4 100μF Electrolytic Capacitor Installation

LiPo disconnected before installation. The 100μF electrolytic capacitor placed in parallel with the Buck output, polarity confirmed (long lead → OUT+, short lead with white stripe → OUT−). LiPo reconnected and output voltage re-measured.

**Pass criteria:** output voltage maintained at 5.00V ± 0.1V after capacitor installation. No abnormal heating or odor from the capacitor.
**Result:** Pass. Detailed numerical value recorded in the Notion test log.

### 0-5 Breadboard Power Rail Migration

The breadboard left power rail was reserved for 7.4V (motor supply, currently unconnected) and the right rail for 5V (Buck output, sensor/logic supply). The HC-SR04 module was placed horizontally on the breadboard with jumper wires routed across to the right 5V rail. The previous USB 5V jumper was removed; Buck OUT+ wired to the right VCC rail and Buck OUT− to the GND rail. NUCLEO GND was tied to the same GND rail to ensure a common ground reference between the MCU (USB-powered) and the sensor circuit (Buck-powered). Power-on sequence: NUCLEO USB connected first (LD1 LED confirmed), then LiPo connected to activate the Buck output.

The horizontal HC-SR04 placement crosses the unused 7.4V rail with jumper wires. Since the 7.4V rail is unenergized at this stage (motor supply not yet connected), no noise coupling risk is present. This routing must be revisited before Phase 4-B (motor + sensor concurrent noise test) where the 7.4V rail will be live.

**Pass criteria:** VCC rail = 5.00V ± 0.1V, NUCLEO LD1 LED on, no abnormal heating.
**Result:** Pass. VCC rail measured 4.9 ~ 5.0V. NUCLEO operating normally.

### 0-6 Re-verification of Phase 3 (HC-SR04) Under Buck 5V

The unmodified Phase 3 firmware (TIM3 Input Capture, interrupt-driven echo) was used. The HC-SR04 was aimed at a target placed exactly 300mm away, measured with a tape ruler. Distance log was streamed via PuTTY at 115200 baud and saved.

The log file contains two segments separated by an MCU reset: pre-reset measurements were taken aiming at empty space (no reflector) and are not part of the verification data; post-reset measurements (181 samples) constitute the actual 300mm verification dataset.

**Pass criteria:** measured distance consistent with Phase 3 results (USB 5V), no abnormal noise behavior introduced by the Buck supply.
**Result:** Pass. See detailed analysis below.

## 0-6 Detailed Analysis

### Main cluster (280 ~ 305mm, 95.6% of post-reset samples)

| Metric | Value |
|---|---|
| Sample count | 173 / 181 (95.6%) |
| Target distance | 300.00 mm |
| Mean | 295.91 mm |
| Error from target | −4.09 mm |
| Standard deviation | 2.63 mm |
| Range (min ~ max) | 291.90 ~ 300.30 mm |

### Outliers (3.9% of samples)

7 samples (3.9%) fell into the 171 ~ 199mm range. This pattern is consistent with the known HC-SR04 second-echo / missed-echo behavior (occasional shortened pulses caused by reflector geometry and acoustic interference) and is independent of the power supply. The outlier rate is within the acceptable range observed in Phase 3.

### Loop timing observation

Pre-reset log records contained a `loop_max` field reading 4271111493 μs (consistent with a uint32_t underflow / wrap-around pattern). After the MCU reset, `loop_max` stabilized in the 5131 ~ 5228 μs range across all 181 post-reset samples. The pre-reset glitch is not present in the verification dataset and does not affect the Phase 0 conclusion. The DWT-based loop-time measurement logic should be reviewed during Phase 6 (sensor + KF integration, 200 Hz loop verification) as a follow-up.

### Comparison with Phase 3 (USB 5V)

| Metric | Phase 3 (USB 5V) | Phase 0-6 (Buck 5V) |
|---|---|---|
| Target distance | 297 mm (A4 paper setup) | 300 mm (tape ruler) |
| Measurement environment | A4 sheet placement | Tape-measured distance |
| Mean error | systematic ~−19 mm offset | −4.09 mm |
| Power source | NUCLEO USB 5V | Buck 5V |

The improved accuracy (smaller offset) in 0-6 is primarily attributable to the more precise physical setup (tape-measured target distance) rather than to any change introduced by the Buck supply. The relevant verification result is that the Buck supply did **not introduce any degradation** in measurement stability or behavior compared to USB 5V operation.

## Conclusion

All Phase 0 sub-tests (0-1 through 0-6) passed. The Buck Converter and LiPo 2S combination provides a stable 5V rail under both no-load and HC-SR04 sensor load conditions. The migration from NUCLEO USB 5V to Buck 5V did not introduce measurement noise or instability for the previously verified HC-SR04 distance measurement. The 5V rail is now confirmed as the production power source for all subsequent sensor integration phases.

## Open Items for Subsequent Phases

- **4.7kΩ pull-up resistors (×2)** for I2C SCL/SDA pulled to 3.3V are not yet acquired. Required for Phase 1 (VL53L0X). Phase 1 cannot start until these arrive.
- **HC-SR04 horizontal placement** on the breadboard crosses the (currently unenergized) 7.4V rail. Re-routing required before Phase 4-B (motor + sensor concurrent noise test).
- **Loop-time measurement glitch** (`loop_max` underflow observed once before reset) to be reviewed during Phase 6 (200 Hz loop integration).
- **Encoder re-verification under Buck 5V** was not performed in Phase 0. Since the encoder shares the same 5V rail and consumes minimal current, behavior under Buck 5V is expected to match the Phase 2 USB 5V result. Re-verification can be folded into Phase 7 (full system integration) without a dedicated test step.
