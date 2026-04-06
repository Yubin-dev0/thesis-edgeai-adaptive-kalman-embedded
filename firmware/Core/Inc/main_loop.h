/**
 * @file    main_loop.h
 * @brief   Control loop public interface
 */
#ifndef MAIN_LOOP_H
#define MAIN_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Initialize peripherals and DWT, send CSV header.
 *         Call after all HAL_Init / MX_xxx_Init() in main().
 */
void ControlLoop_Init(void);

/**
 * @brief  Enter infinite control loop (never returns).
 *         Sleeps via WFI between 5ms TIM6 ticks.
 */
void ControlLoop_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_LOOP_H */
