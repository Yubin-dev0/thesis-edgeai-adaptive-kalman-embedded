# Phase 0: Power Supply Verification Report

**Date**: 2026-04-29
**Board**: NUCLEO-F446RE (STM32F446RET6)
**Power chain**: 2S LiPo (7.4V) → YwRobot PWR060010 Buck Converter (LM2596, fixed 5V) → 5V rail

## Confirmed Parameters

| Parameter | Specification |
|-----------|---------------|
| Battery | 2S LiPo, 7.4V nominal (8.4V full / 6.4V cutoff) |
| Buck Converter | YwRobot PWR060010 (LM2596), fixed 5V output, screw terminals |
| Decoupling (Buck output) | 100 µF electrolytic + 100 nF ceramic, parallel |

## Test Results

| # | Test | Result | Pass/Fail |
|---|------|--------|-----------|
| 0-1 | LiPo cell 1 voltage | 3.9 V | Pass (≥ 3.7 V) |
| 0-1 | LiPo cell 2 voltage | 3.9 V | Pass (≥ 3.7 V) |
| 0-1 | LiPo cell balance | 0.00 V difference | Pass (≤ 0.05 V) |
| 0-1 | LiPo pack voltage (main output) | 7.4 V | Pass (7.4–8.4 V) |
| 0-2 | Buck Converter no-load output | 5.0–5.1 V | Pass (5.0 ± 0.1 V) |
| 0-4 | Decoupling capacitor installation | No voltage change observed (expected) | Pass |
| 0-3 | Buck Converter loaded output | Deferred to Phase 4-A (motor as load) | — |

## Wiring

| Source | Destination | Notes |
|--------|-------------|-------|
| LiPo (+) red | Buck VIN | screw terminal |
| LiPo (-) black | Buck GND (input side) | screw terminal |
| Buck VCC | Breadboard +5V rail | screw terminal |
| Buck GND (output side) | Breadboard GND rail | screw terminal |
| 100 µF electrolytic | Across +5V / GND rails | polarity: long leg = + |
| 100 nF ceramic | Across +5V / GND rails | non-polar |

## Setup Photo

See `logs/power_setup_overview.jpg` for the breadboard setup with LiPo, 
Buck Converter, and multimeter probe in measurement position.

## Notes

- **YwRobot PWR060010 has a fixed 5 V output** — no trimmer adjustment is required or possible. Output measured 5.0–5.1 V, which is within the LM2596 typical accuracy.
- **Pack voltage (7.4 V) reads slightly below the sum of cells (7.8 V)** due to multimeter input impedance loading the balance lead. Difference is within expected measurement tolerance and does not indicate a problem.
- **Loaded test deferred** — the Buck output under load will be verified during Phase 4-A motor test, where the motor driver acts as a realistic load (peak current up to 1.2 A per channel).
- **Decoupling layout** — 100 µF electrolytic and 100 nF ceramic placed in parallel at the Buck output, providing low-frequency bulk capacitance and high-frequency filtering respectively.

## Safety Reminders

- LiPo polarity must be verified before connecting to Buck VIN. Reverse polarity will damage the LM2596 module.
- 100 µF electrolytic capacitor is polarized. Long leg goes to (+), striped side goes to (-).
- LiPo should be stored in a fire-resistant LiPo safety bag when not in use.
