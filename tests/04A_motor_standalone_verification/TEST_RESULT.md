# Phase 4-A: TB6612FNG Motor Driver Standalone Verification Report

**Date**: 2026-05-03
**Board**: NUCLEO-F446RE (STM32F446RET6)
**Motor Driver**: JMOD-MOTOR-1 (TB6612FNG, JCnet)
**Motor**: FIT0450 (1 unit, M1 / A-channel only)
**Power chain**: 2S LiPo (7.4V) → JMOD VIN/VCC (motor power), NUCLEO 5V → JMOD +5V (logic)

## Confirmed Parameters

| Parameter | Specification |
|-----------|---------------|
| PWM Timer | TIM1 CH1 (PA8), 10 kHz, 1000-step duty resolution |
| PWM Prescaler / Period | 17 / 999 (180MHz APB2 → 10MHz → 10kHz PWM) |
| Direction control | GPIO PC8 (AIN1), PC9 (AIN2) |
| Standby control | GPIO PC12 (STBY) |
| Serial interface | USART2 (PA2/PA3) via ST-LINK Virtual COM Port, 115200 baud |
| Motor power supply | LiPo 7.4V direct to JMOD VIN (jumper-bridged to VCC) |
| Logic power supply | NUCLEO 5V (USB-powered) to JMOD +5V |
| Boot safety state | STBY=LOW, PWM=0% (motor disabled until 'i' command) |

## Pin Wiring

### NUCLEO ↔ JMOD-MOTOR-1
| Wire color | NUCLEO pin | JMOD pin | Function |
|------------|------------|----------|----------|
| Black | GND | GND (left header) | Common ground |
| Yellow | PA8 (D7, TIM1_CH1) | PWMA | A-channel PWM |
| Orange | PC8 | AIN1 | Direction bit 1 |
| Purple | PC9 | AIN2 | Direction bit 2 |
| Green | PC12 | STBY | Driver enable |
| Red | 5V (Arduino header) | +5V | Logic power |

### JMOD power and motor output
| Source | Destination | Notes |
|--------|-------------|-------|
| Breadboard 7.4V rail | JMOD VIN | Motor power input |
| Breadboard 7.4V rail | JMOD VCC | Bridged via breadboard rail (bypasses on-board jumper) |
| JMOD AO1 / AO2 | FIT0450 motor terminals | Motor output (encoder leads not connected for this test) |

**Note on VCC bridging**: The on-board VSEL jumper (5V / VCC / VIN) is left unconfigured. Instead, both VIN and VCC pins are connected to the same breadboard 7.4V rail via separate jumper wires. This is electrically equivalent to bridging VCC↔VIN with a jumper cap and provides more reliable contact than a single jumper wire spanning the two pins.

## Test Results

| # | Test | Result | Pass/Fail |
|---|------|--------|-----------|
| 4A-1 | Build firmware with motor control logic | 0 errors, 0 warnings | Pass |
| 4A-2 | Flash via ST-LINK and verify boot serial output | Boot banner received in PuTTY | Pass |
| 4A-3 | Boot safety state | STBY=LOW, PWM=0% on power-up | Pass |
| 4A-4 | Reject unauthorized commands before init | `s` before `i` returns "ERR: run 'i' first" | Pass |
| 4A-5 | Init command (`i`) sets STBY=HIGH | Status reports `STBY=H, dir=FWD, PWM=0` | Pass |
| 4A-6 | PWM signal output at PWMA pin | 0.98 V DC average at 30% duty (expected ~1.0 V) | Pass |
| 4A-7 | Direction signal at AIN1, AIN2 | AIN1=3.3V, AIN2=0V (forward direction) | Pass |
| 4A-8 | JMOD logic supply rails | VIN=7.4V, +5V=5.0V | Pass |
| 4A-9 | Motor output at AO1, AO2 | Confirmed > 0V after VCC supply restored | Pass |
| 4A-10 | SoftStart command (`s`) drives motor | Motor rotates smoothly from 0 to 30% over 1 second | Pass |
| 4A-11 | Stop command (`x`) disables driver | STBY=LOW, motor coasts to stop | Pass |

## Debugging Notes

A multi-stage diagnostic was required to identify the root cause of the initial no-rotation symptom. The signal chain was traced from MCU outputs through JMOD inputs to motor output terminals:

| Measurement point | Value | Conclusion |
|-------------------|-------|------------|
| MCU PWM output (JMOD PWMA pin) | 0.98 V | PWM signal correctly delivered |
| STBY input | 3.3 V | Enable signal present |
| AIN1 / AIN2 inputs | 3.3 V / 0 V | Direction logic correct (forward) |
| VIN input | 7.4 V | Motor supply rail present at VIN pin |
| **VCC input** | **0 V (initial) → 7.4 V (after fix)** | **Root cause: VCC was not bridged to VIN** |
| Motor direct LiPo test | Rotation confirmed | Motor itself is functional |

**Root cause**: The TB6612FNG chip's internal motor power input (VM) is connected to the VCC pin, not VIN directly. The VSEL jumper on the JMOD module is responsible for bridging VIN↔VCC (or 5V↔VCC). With VCC left floating, the chip received all control signals but had no motor supply voltage to switch onto AO1/AO2.

**Resolution**: Connected JMOD VCC to the 7.4V breadboard rail with a dedicated jumper wire, parallel to the existing VIN connection. This restored AO1/AO2 output and enabled motor rotation.

## Notes

- **PWM frequency** of 10 kHz is well above the audible range and below the TB6612FNG's stated maximum (100 kHz), giving smooth current control without acoustic noise.
- **PWM_MAX hard cap** is set to 80% in firmware as an additional safety margin during initial bring-up. This can be raised to 100% later if higher torque is required.
- **Encoder leads** of the FIT0450 (yellow/blue/white) are intentionally left disconnected for this test. Encoder integration is verified separately in Phase 2.
- **B-channel** (PWMB, BIN1, BIN2, BO1, BO2) is intentionally unwired for this test. M2 verification will follow with identical methodology.
- **Loaded supply test** deferred from Phase 0 is implicitly satisfied here: with the motor running at 30% duty, the Buck Converter 5V rail and the 7.4V LiPo rail showed no observable instability, and the firmware continued to respond to serial commands without resets.
- **Test firmware** is preserved at `main_motor_test.c` in this directory. 
  To reproduce this test, replace `firmware/Core/Src/main.c` with this file 
  and rebuild. Original main.c must be backed up first.

## Safety Reminders

- Always set STBY=LOW (`x` command) before connecting or disconnecting motor leads.
- Connect LiPo last in the power-on sequence; disconnect first in the power-off sequence.
- Keep multimeter probes away from motor terminals while motor is energized.
- LiPo must be stored in a fire-resistant safety bag when not in use.

