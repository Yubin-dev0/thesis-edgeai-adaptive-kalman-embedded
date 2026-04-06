/**
 * @file    main_loop.c
 * @brief   200Hz (5ms) Control Loop Skeleton — STM32F446RE
 * @author  Yubin
 * @date    2026-04-03
 *
 * Pipeline (per 5ms tick):
 *   Phase 1: Encoder read          (~1μs)
 *   Phase 2: KF Prediction         (~5μs)
 *   Phase 3: VL53L0X Update        (every 4th loop, ~200μs)
 *   Phase 4: HC-SR04 Trigger       (every 10th loop)
 *   Phase 5: Motor PWM update      (~5μs)
 *   Phase 6: CSV logging via DMA   (every 4th loop = 50Hz)
 *
 * DMA double-buffering: buf[0] transmitting while buf[1] is written, then swap.
 * DWT cycle counter used for loop profiling.
 */

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ── Hardware handles (extern from CubeMX-generated main.c) ─────────── */
extern TIM_HandleTypeDef htim6;   /* 5ms interrupt */
extern TIM_HandleTypeDef htim1;   /* Motor PWM (CH1, CH2) */
extern TIM_HandleTypeDef htim2;   /* Encoder L */
extern TIM_HandleTypeDef htim3;   /* Encoder R */
extern UART_HandleTypeDef huart2; /* CSV logging (DMA TX) */
extern ADC_HandleTypeDef hadc1;   /* Battery voltage */

/* ── Constants ──────────────────────────────────────────────────────── */
#define LOOP_FREQ_HZ        200
#define DT_S                (1.0f / LOOP_FREQ_HZ)  /* 0.005 */

#define VL53L0X_DIVIDER     4    /* 200/4 = 50Hz */
#define HCSR04_DIVIDER      10   /* 200/10 = 20Hz */
#define CSV_LOG_DIVIDER     4    /* 200/4 = 50Hz */

#define ENCODER_PPR         960  /* FIT0450 pulses per revolution */
#define WHEEL_CIRCUMFERENCE_MM  (65.0f * 3.14159265f) /* 65mm wheel */

#define BATT_LOW_THRESHOLD_MV   6400  /* 6.4V → motor cutoff */
#define ADC_VREF_MV             3300
#define ADC_RESOLUTION          4096
/* Voltage divider ratio: if using 10k/10k divider → factor = 2 */
#define BATT_DIVIDER_FACTOR     2

#define DMA_BUF_SIZE        256

/* ── DMA Double Buffer ──────────────────────────────────────────────── */
static char dma_buf[2][DMA_BUF_SIZE];
static volatile uint8_t dma_write_idx = 0;  /* index we write into */
static volatile bool     dma_tx_busy  = false;

/* ── Loop State ─────────────────────────────────────────────────────── */
static volatile bool loop_tick = false;  /* set in TIM6 ISR */
static uint32_t loop_counter  = 0;

/* ── Sensor / State Variables ───────────────────────────────────────── */
typedef struct {
    /* Encoders */
    int32_t  enc_left_raw;
    int32_t  enc_right_raw;
    int32_t  enc_left_delta;
    int32_t  enc_right_delta;
    float    vel_left_mm_s;
    float    vel_right_mm_s;

    /* KF state (placeholder — replace with your KF struct) */
    float    kf_x;        /* estimated position mm */
    float    kf_v;        /* estimated velocity mm/s */

    /* VL53L0X */
    bool     vl53_new_data;
    uint16_t vl53_range_mm;

    /* HC-SR04 */
    bool     hcsr04_new_data;
    uint16_t hcsr04_range_mm;

    /* Motor */
    float    pwm_left;    /* -1.0 … +1.0 */
    float    pwm_right;

    /* Battery */
    uint16_t batt_mv;
    bool     batt_low;

    /* Profiling */
    uint32_t loop_cycles;
    uint32_t loop_max_cycles;
} ControlState;

static ControlState cs;

/* ════════════════════════════════════════════════════════════════════
 *  DWT Cycle Counter
 * ════════════════════════════════════════════════════════════════════ */
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/* ════════════════════════════════════════════════════════════════════
 *  Phase 1: Encoder Read
 * ════════════════════════════════════════════════════════════════════ */
static void Phase1_EncoderRead(void)
{
    /* Read 16-bit timer counters (auto-reload handles overflow) */
    int32_t cnt_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t cnt_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    cs.enc_left_delta  = cnt_l - cs.enc_left_raw;
    cs.enc_right_delta = cnt_r - cs.enc_right_raw;
    cs.enc_left_raw    = cnt_l;
    cs.enc_right_raw   = cnt_r;

    /* Convert to velocity: delta_pulses / dt * (circumference / PPR) */
    const float pulse_to_mm = WHEEL_CIRCUMFERENCE_MM / (float)ENCODER_PPR;
    cs.vel_left_mm_s  = (float)cs.enc_left_delta  * pulse_to_mm / DT_S;
    cs.vel_right_mm_s = (float)cs.enc_right_delta * pulse_to_mm / DT_S;
}

/* ════════════════════════════════════════════════════════════════════
 *  Phase 2: Kalman Filter — Prediction
 * ════════════════════════════════════════════════════════════════════ */
static void Phase2_KF_Predict(void)
{
    /*
     * TODO: Replace with actual KF/AKF predict step.
     * Placeholder: simple integration.
     */
    float v_avg = (cs.vel_left_mm_s + cs.vel_right_mm_s) * 0.5f;
    cs.kf_x += v_avg * DT_S;
    cs.kf_v  = v_avg;
}

/* ════════════════════════════════════════════════════════════════════
 *  Phase 3: VL53L0X — Check + KF Update
 * ════════════════════════════════════════════════════════════════════ */
static void Phase3_VL53L0X_Update(void)
{
    if ((loop_counter % VL53L0X_DIVIDER) != 0)
        return;

    /*
     * TODO: Non-blocking I2C read. For now, dummy.
     * In real code: check I2C DMA complete flag, read result register.
     */
    cs.vl53_new_data = true;       /* dummy: pretend new data */
    cs.vl53_range_mm = 150;        /* dummy value */

    if (cs.vl53_new_data) {
        /*
         * TODO: KF Update step with vl53_range_mm as measurement.
         * Placeholder: direct assignment.
         */
        cs.kf_x = (float)cs.vl53_range_mm;
        cs.vl53_new_data = false;
    }
}

/* ════════════════════════════════════════════════════════════════════
 *  Phase 4: HC-SR04 — Trigger
 * ════════════════════════════════════════════════════════════════════ */
static void Phase4_HCSR04_Trigger(void)
{
    if ((loop_counter % HCSR04_DIVIDER) != 0)
        return;

    /*
     * TODO: Trigger pulse on GPIO (10μs HIGH).
     * Echo capture via input-capture timer ISR → hcsr04_range_mm.
     * For now, dummy.
     */
    cs.hcsr04_new_data = true;
    cs.hcsr04_range_mm = 300;     /* dummy */
}

/* ════════════════════════════════════════════════════════════════════
 *  Phase 5: Motor PWM Update
 * ════════════════════════════════════════════════════════════════════ */
static void Phase5_MotorPWM(void)
{
    if (cs.batt_low) {
        /* Safety cutoff */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        /* TODO: Also set TB6612 STBY pin LOW */
        return;
    }

    /*
     * TODO: Replace with PID output → PWM.
     * pwm_left/right in range [-1.0, +1.0].
     * TIM1 ARR assumed = 999 → duty 0..999.
     */
    uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    /* Clamp */
    float pl = cs.pwm_left;
    float pr = cs.pwm_right;
    if (pl >  1.0f) pl =  1.0f;
    if (pl < -1.0f) pl = -1.0f;
    if (pr >  1.0f) pr =  1.0f;
    if (pr < -1.0f) pr = -1.0f;

    /* Direction via TB6612 AIN1/AIN2, BIN1/BIN2 GPIOs */
    /* TODO: set direction GPIOs based on sign */

    uint16_t duty_l = (uint16_t)(fabsf(pl) * (float)arr);
    uint16_t duty_r = (uint16_t)(fabsf(pr) * (float)arr);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_l);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_r);
}

/* ════════════════════════════════════════════════════════════════════
 *  Phase 6: CSV Logging (DMA Double Buffer)
 * ════════════════════════════════════════════════════════════════════ */
static void Phase6_CSV_Log(void)
{
    if ((loop_counter % CSV_LOG_DIVIDER) != 0)
        return;

    /* Skip if previous DMA TX still in progress */
    if (dma_tx_busy)
        return;

    char *buf = dma_buf[dma_write_idx];

    int len = snprintf(buf, DMA_BUF_SIZE,
        "%lu,%.1f,%.1f,%.1f,%.1f,%u,%u,%.2f,%.2f,%u,%lu\r\n",
        HAL_GetTick(),
        cs.vel_left_mm_s,
        cs.vel_right_mm_s,
        cs.kf_x,
        cs.kf_v,
        cs.vl53_range_mm,
        cs.hcsr04_range_mm,
        cs.pwm_left,
        cs.pwm_right,
        cs.batt_mv,
        cs.loop_cycles
    );

    if (len > 0 && len < DMA_BUF_SIZE) {
        dma_tx_busy = true;
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)buf, (uint16_t)len);
        /* Swap buffer index for next write */
        dma_write_idx ^= 1;
    }
}

/* DMA TX complete callback — clears busy flag */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart2.Instance) {
        dma_tx_busy = false;
    }
}

/* ════════════════════════════════════════════════════════════════════
 *  Battery ADC Read (non-blocking polling, ~1μs)
 * ════════════════════════════════════════════════════════════════════ */
static void Battery_Check(void)
{
    /* Read once every 200 loops (1Hz) to reduce overhead */
    if ((loop_counter % 200) != 0)
        return;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(&hadc1);
        cs.batt_mv = (uint16_t)((raw * ADC_VREF_MV * BATT_DIVIDER_FACTOR) / ADC_RESOLUTION);
        cs.batt_low = (cs.batt_mv < BATT_LOW_THRESHOLD_MV);
    }
    HAL_ADC_Stop(&hadc1);
}

/* ════════════════════════════════════════════════════════════════════
 *  TIM6 ISR — sets tick flag (called from stm32f4xx_it.c)
 * ════════════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        loop_tick = true;
    }
}

/* ════════════════════════════════════════════════════════════════════
 *  CSV Header (send once at boot)
 * ════════════════════════════════════════════════════════════════════ */
static void CSV_SendHeader(void)
{
    const char *hdr = "tick_ms,vel_L,vel_R,kf_x,kf_v,vl53_mm,hcsr04_mm,"
                      "pwm_L,pwm_R,batt_mV,loop_cyc\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)hdr, strlen(hdr), 100);
}

/* ════════════════════════════════════════════════════════════════════
 *  Init & Main Loop  —  call from main() after CubeMX init
 * ════════════════════════════════════════════════════════════════════ */
void ControlLoop_Init(void)
{
    memset(&cs, 0, sizeof(cs));

    DWT_Init();

    /* Start encoder timers */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* Start motor PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

    CSV_SendHeader();

    /* Start 5ms tick timer */
    HAL_TIM_Base_Start_IT(&htim6);
}

void ControlLoop_Run(void)
{
    /* Non-blocking main loop — WFI sleeps between ticks */
    while (1) {
        if (!loop_tick) {
            __WFI();  /* sleep until next interrupt */
            continue;
        }
        loop_tick = false;

        uint32_t t0 = DWT_GetCycles();

        /* ── Pipeline ─────────────────────────────── */
        Phase1_EncoderRead();
        Phase2_KF_Predict();
        Phase3_VL53L0X_Update();
        Phase4_HCSR04_Trigger();
        Battery_Check();
        Phase5_MotorPWM();
        Phase6_CSV_Log();

        /* ── Profiling ────────────────────────────── */
        cs.loop_cycles = DWT_GetCycles() - t0;
        if (cs.loop_cycles > cs.loop_max_cycles) {
            cs.loop_max_cycles = cs.loop_cycles;
        }

        loop_counter++;

        /* Overrun check: if next tick already pending, we missed deadline */
        if (loop_tick) {
            /* TODO: increment overrun counter, log error */
        }
    }
}
