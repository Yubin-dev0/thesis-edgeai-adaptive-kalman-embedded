# Phase 2: Encoder Verification Report

**Date**: 2026-04-06  
**Board**: NUCLEO-F446RE (STM32F446RET6)  
**Power**: USB 5V (no motor drive)

## Confirmed Parameters

```c
#define ENCODER_PPR         8
#define GEAR_RATIO          120
#define PULSES_PER_REV      3840    // 8 * 120 * 4 (x4 quadrature)
#define WHEEL_DIAMETER_MM   66.0f   // from datasheet
#define MM_PER_PULSE        0.05397 // π × 66 / 3840
```

## Test Results

| # | Test | Result | Pass/Fail |
|---|------|--------|-----------|
| 2-1 | Manual rotation counter response | Both M1 and M2 increment/decrement correctly | Pass |
| 2-2 | Pulses per revolution (M1, TIM2) | ~3870 pulses (expected 3840, error +0.8%) | Pass |
| 2-2 | Pulses per revolution (M2, TIM4) | ~3891 pulses (expected 3840, error +1.3%) | Pass |
| 2-3 | mm/pulse conversion factor | 0.05397 mm/pulse | Pass |
| 2-4 | 105mm linear travel verification (M1) | 103.4–103.5mm measured (error −1.5%) | Pass |
| 2-4 | 105mm linear travel verification (M2) | 106.3–107.3mm measured (error +1.2–2.2%) | Pass |

## Pin Assignment

| Function | Pin | Peripheral |
|----------|-----|------------|
| Encoder 1 CH_A | PA15 | TIM2_CH1 |
| Encoder 1 CH_B | PB3 | TIM2_CH2 |
| Encoder 2 CH_A | PB6 | TIM4_CH1 |
| Encoder 2 CH_B | PB7 | TIM4_CH2 |
| Debug UART TX | PA2 | USART2 |
| Debug UART RX | PA3 | USART2 |

## Troubleshooting Notes

- **E5V pin is for external power input only** — when using USB power, use the 5V pin on the Arduino header or CN7 pin 18 instead.
- **IWDG causes continuous resets** if not refreshed periodically — disabled during encoder-only testing.
- **PA15 requires SWD-only debug mode** — JTAG must be disabled in SYS settings to free this pin for TIM2_CH1.
- **Encoder uses open-collector output** — internal pull-up resistors configured via GPIO init code since CubeIDE does not allow pull-up settings for timer-mapped pins.
