/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : E4 Static Long-Term Stability measurement firmware
  *                    Dual KF (Fixed + CM-AKF) + TinyML, 30-min static logging,
  *                    28-col CSV (unchanged schema from E3)
  *
  * [E4 PATCH 2026-05-22] changes vs E3:
  *   - CSV_SCENARIO_ID: 3U -> 4U (E4 Static Long-Term Stability)
  *   - PHASE6_N_TEST_LOOPS: 1000 -> 360000  (200Hz × 1800s = 30min)
  *   - Banner labels: "E3 Dynamic Occlusion" -> "E4 Static Stability"
  *   - tinyml_infer_us field meaning: max-so-far -> per-row LAST infer time.
  *     Required for the 30-min latency time-series analysis (R̂ drift,
  *     inference latency accumulation, RQ1 long-term stability check).
  *     CSV column name unchanged -> Dayoung's analysis page / file
  *     naming convention (E4_run{N}_{algo}.csv) is unaffected.
  *   - CSV header: 28 columns unchanged.
  *   - Setup: robot stationary (motors OFF), wall at ~500mm, white foam
  *     board (same surface as E1 baseline, per thesis §4.2 — E4 controls
  *     surface variable to isolate long-term factors: battery drift,
  *     thermal effects, R-hat drift, memory leaks).
  *   - 3 runs × 30min per thesis [표 4-4]; B1 trigger excludes pre-run
  *     transient (HC-06 connect, operator settle).
  *
  * [E3 PATCH 2026-05-20] changes vs E2:
  *   ... (existing comments preserved) ...
  ******************************************************************************
  ******************************************************************************
  * @file           : main.c
  * @brief          : E3 Dynamic Occlusion measurement firmware
  *                    Dual KF (Fixed + CM-AKF), B1 trigger, HC-SR04, 25-col CSV
  *
  **
  * [E3 PATCH 2026-05-20] changes vs E2:
  *   - CSV_SCENARIO_ID: 2U -> 3U (E3 Dynamic Occlusion)
  *   - Banner/result labels: "E2 Reflectivity" -> "E3 Dynamic Occlusion"
  *   - Scenario: aluminum panel manually inserted into ToF beam path
  *     at ~100mm wall distance for <1s when robot reaches 250mm.
  *     Note: thesis 4.2 statement on "no manual occlusion" needs
  *     section update — change of method made after empirical
  *     confirmation that black foam board fails to produce
  *     range_status != 0.
  *   - TinyML #if 0 unchanged (still disabled, awaiting stage 4-C).
  *
  * [E2 PATCH 2026-05-20] changes vs E1 baseline firmware:
  *   - CSV_SCENARIO_ID: 1U -> 2U (E2)
  *   - Banner/result labels: "E1 Baseline" -> "E2 Reflectivity"
  *   - TinyML 4-B-2 inference call disabled (#if 0 ... #endif).
  *     Reason: ai_init() currently fails (ai_infer_count==0) and
  *     leaving the disabled inference path in place corrupts UART
  *     integrity (~98% vs the clean 100% seen in E1 run01~05).
  *     The call will be re-enabled in stage 4-C together with the
  *     kf_tinyml instance, 28-column CSV, and ai_init() fix.  Until
  *     then this firmware is a Fixed/CM-only logger, which is
  *     exactly what E2 learning-data collection needs.
  *   - ai_init() is still called at boot.  The STAI runtime is
  *     linked in but never invoked, so the timing instrumentation
  *     prints nothing (ai_infer_count==0 -> guarded printf is
  *     skipped).  This is the intended state for E2.
  *
  * Changes from Phase 6 Step 6 v2:
  *   - PHASE6_N_TEST_LOOPS: 360000 -> 1000 (E1 run length, ~5s @ 200Hz)
  *   - CSV_SCENARIO_ID: 0 -> 1 (E1). Change this #define per scenario.
  *   - B1 (PC13) button trigger: measurement counting waits for B1 press,
  *     so the stationary transient before manual roll is excluded.
  *   - HC-SR04 integrated: PA1 trigger pulse every 50ms, echo via TIM3 CH1
  *     interrupt, distance = echo_us * 0.1715.
  *   - Dual Kalman Filter: kf_fixed (use_akf=false) and kf_cm (use_akf=true)
  *     run on the same ToF/encoder input. Both estimates logged.
  *   - CSV expanded to 25 columns (12 shared + 6 fixed + 7 cm).
  *
  * [SCHEME C] predict/update time-structure fix (2026-05-19)
  *   PROBLEM (diagnosed from E1_run00 CSV):
  *     kf_predict ran every 200Hz loop, kf_update only on ToF DataReady
  *     (~50Hz, but actually 18-62ms jitter + 11 missed frames / 234).
  *     predict count between two updates was variable (4..~12), and
  *     predict/update were not phase-aligned.  This injected a
  *     velocity-proportional negative bias into the residual
  *     (moving-segment fixed_residual mean = -8.57mm), which the
  *     fixed KF tolerated but which drove CM-AKF's R from 24 to the
  *     10000 clamp (positive-feedback divergence).
  *   FIX:
  *     The KF is no longer stepped every loop.  Each 200Hz loop only
  *     ACCUMULATES encoder pulse deltas.  When a ToF measurement is
  *     ready, exactly ONE kf_predict (fed the whole accumulated
  *     displacement) followed by ONE kf_update is performed, then the
  *     accumulator is reset.  predict:update is now strictly 1:1 and
  *     phase-aligned, matching cm_akf_1D.py.  kalman_filter.c/.h are
  *     UNCHANGED, so E0/Phase 6 equivalence is preserved.
  *   The 200Hz main loop itself is unchanged (sensors, HC-SR04, CSV,
  *     timing instrumentation all still run at 200Hz) -> RQ1 intact.
  *
  * [F5 FIX] tof_meas_rate definition corrected (2026-05-19)
  *   The CSV field tof_meas_rate previously held the step-to-step ToF
  *   distance change-rate.  Per thesis Table 3-1, F5 Measurement Rate
  *   is the ratio of range_status==0 within a W=20 sliding window of
  *   ToF measurements.  It is now computed that way (circular buffer
  *   of the last 20 range_status values).  The CSV column name
  *   tof_meas_rate is kept for compatibility with the analysis tools;
  *   only its meaning changed.
  *
  * Encoder sign note:
  *   R-side encoder is inverted at one point:
  *     int16_t dr = -(enc_r_now - enc_r_prev);
  *   This corrects enc_r_total, pos_r_mm and the KF input at once.
  *
  * IWDG: 8s timeout (dev). Note B1 wait loop refreshes IWDG so a long
  *       wait before the button press does not trigger a reset.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "kalman_filter.h"

#include "stai.h"
#include "network.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PHASE6_N_TEST_LOOPS     360000U   /* E4: 200Hz x 1800s = 30 min       */
#define PHASE6_LOOP_BUDGET_US   4500U

#define VL53L0X_TIMING_BUDGET_US    20000U
#define VL53L0X_INTER_PERIOD_MS     20U

#define MM_PER_PULSE            0.05397f
#define LOOP_DT_SEC             0.005f

/* [SCHEME C] KF input scaling.
 * kf_predict does  x_pred = x + KF_B * u  with KF_B = 0.005 (= dt).
 * In Scheme C one predict consumes the WHOLE displacement D_mm
 * accumulated since the previous ToF update, so we need
 *   KF_B * u = D_mm   =>   u = D_mm / KF_B.
 * D_mm itself is (accumulated pulses) * MM_PER_PULSE, so:
 *   u = (accum_pulses * MM_PER_PULSE) / KF_B.
 * MM_PER_PULSE / KF_B is precomputed here.  This is the value
 * passed (negated) to kf_predict.  Note KF_B is unchanged (0.005),
 * so kalman_filter.h and the E0 simulation stay byte-for-byte
 * equivalent. */
#define PULSES_TO_KF_U          (MM_PER_PULSE / 0.005f)   /* KF_B = 0.005 */

#define LOG_DECIMATION          4U
#define CSV_BUF_SIZE            512U      /* widened for 25-column line       */
#define CSV_SCENARIO_ID         4U        /* (change per scenario) */

#define IWDG_REFRESH_EVERY      10U     /* refresh every 10 loops = 50ms */
#define IWDG_RELOAD_VAL         4000U   /* 4000*64/32000 = 8.0s timeout  */

/* HC-SR04 ---------------------------------------------------------------- */
#define HCSR04_TRIG_PERIOD_MS   50U       /* trigger every 50ms (~20Hz)       */
#define HCSR04_TRIG_PULSE_US    10U       /* 10us trigger pulse               */
#define HCSR04_US_TO_MM         0.1715f   /* echo_us -> mm  (343 m/s / 2)     */
#define HCSR04_TRIG_PORT        GPIOA
#define HCSR04_TRIG_PIN         GPIO_PIN_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

volatile uint8_t  echo_capture_state = 0;
volatile uint32_t echo_rising_tick   = 0;
volatile uint32_t echo_falling_tick  = 0;
volatile uint32_t echo_pulse_us      = 0;
volatile uint8_t  echo_ready         = 0;

VL53L0X_Dev_t  vl53l0x_dev;
VL53L0X_DEV    pVL53L0X = &vl53l0x_dev;

volatile uint8_t  loop_flag         = 0;
volatile uint32_t loop_tick         = 0;
volatile uint32_t isr_overrun_count = 0;

static char  csv_tx_buf[CSV_BUF_SIZE];

/* TinyML 4-C: third KF instance + INT8 I/O buffers */
static KalmanFilter kf_tinyml;
static int8_t  ai_input_q[6];    /* INT8 quantized input  */
static int8_t  ai_output_q[1];   /* INT8 quantized output */
static float   ai_last_R_pred = 24.0f;  /* dequantized R (init = Fixed R) */

/* TinyML 4-C-2: standard normalization (mean, std) — from normalization_params.json
 * Method: standard (x_norm = (x - mean) / std)
 * Fit on: E1 Run 1-3 (f5fixed), per thesis [표 3-3]
 * NOTE: F5 (tof_meas_rate) std=0 in training -> mapped to 1.0 (always yields 0). */
static const float AI_FEAT_MEAN[6] = {
    0.1142364889f,   /* F1 cm_residual       */
    21.1486797333f,  /* F2 cm_residual_var   */
    0.0326167047f,   /* F3 cm_residual_mean  */
    23.4317665100f,  /* F4 sensor_disagree   */
    1.0f,            /* F5 tof_meas_rate     */
    8.6278190613f    /* F6 tof_signal_rate   */
};
static const float AI_FEAT_STD[6] = {
    4.5925402641f,   /* F1 */
    11.7192335129f,  /* F2 */
    0.7663779259f,   /* F3 */
    6.4479146004f,   /* F4 */
    1.0f,            /* F5 (std_safe — original std=0) */
    6.7547807693f    /* F6 */
};

/* TinyML 4-C-2: R clamping (논문 3.4절 CM-AKF와 동일 범위) */
#define AI_R_MIN  1.0f
#define AI_R_MAX  10000.0f

/* Quantization params (from stedgeai report) */
#define AI_IN_SCALE   0.913747072f
#define AI_IN_ZP      31
#define AI_OUT_SCALE  0.038760886f
#define AI_OUT_ZP     (-128)

/* TinyML AI inference - Stage 4-A: init only, no inference yet
 * 4-C-3 fix: stai_network is `typedef uint8_t stai_network` (opaque byte
 *   type). The real context size is STAI_NETWORK_CONTEXT_SIZE from
 *   network.h. We allocate a byte array of that size and use a pointer
 *   to it as the network handle. Aligned per CONTEXT_ALIGNMENT. */
static uint8_t ai_net_ctx[STAI_NETWORK_CONTEXT_SIZE]
    __attribute__((aligned(STAI_NETWORK_CONTEXT_ALIGNMENT)));
static stai_network* ai_net = (stai_network*)ai_net_ctx;
static uint8_t ai_activations[196] __attribute__((aligned(8)));
static float    ai_input[6];
static float    ai_output[1];
static uint8_t  ai_initialized = 0;

/* 4-C-3: I/O buffer pointers obtained from get_inputs/get_outputs after init.
 * These point INTO ai_activations[] (PREALLOCATED flag in network.h). */
static int8_t* ai_input_q_real  = NULL;
static int8_t* ai_output_q_real = NULL;

/* TinyML 4-B-1: dummy inference + DWT timing */
static uint32_t ai_infer_count    = 0;       /* 누적 추론 횟수             */
static uint64_t ai_infer_cycles_sum = 0;     /* E4: uint32 overflows at 30min */
static uint32_t ai_infer_cycles_max = 0;     /* 최악값                     */
static uint32_t ai_last_infer_cycles = 0;    /* E4: 마지막 추론 사이클     */
static float    ai_last_R          = 0.0f;   /* 마지막 추론 결과 (디버그)  */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

void  DWT_Init(void);
void  DWT_Delay_us(uint32_t us);
uint32_t DWT_GetTick_us(void);
int   __io_putchar(int ch);

static void VL53L0X_Setup(void);
static void Safe_Delay_ms(uint32_t ms);   /* HAL_Delay with periodic IWDG refresh */
static void HCSR04_Trigger(void);         /* fire a 10us trigger pulse on PA1     */
static void Wait_For_B1(void);            /* block until B1 pressed (IWDG-safe)   */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* TinyML AI initialization - Stage 4-A
 * 4-C-3 diagnostic: printf at each step to see WHERE init fails.
 * Output appears in the boot banner (before "Setup done. LD2 blinking..."). */
static void ai_init(void)
{
    stai_return_code rc;
    stai_ptr act_ptrs[1] = { (stai_ptr)ai_activations };


    printf("[AI_INIT] step1: stai_network_init...\r\n");
        rc = stai_network_init(ai_net);
        if (rc != STAI_SUCCESS) {
            printf("[AI_INIT] FAIL @ step1 (network_init), rc=%d\r\n", (int)rc);
            return;
        }
        printf("[AI_INIT] step1 OK\r\n");

        printf("[AI_INIT] step2: set_activations...\r\n");
        rc = stai_network_set_activations(ai_net, act_ptrs, 1);
        if (rc != STAI_SUCCESS) {
            printf("[AI_INIT] FAIL @ step2 (set_activations), rc=%d\r\n", (int)rc);
            return;
        }
        printf("[AI_INIT] step2 OK\r\n");

        /* 4-C-3: STAI_FLAG_PREALLOCATED means I/O buffers are inside the
         *        activations area. We must NOT call set_inputs/set_outputs.
         *        Instead, get_inputs/get_outputs returns real addresses
         *        after init; we read those once and cache for inference. */
        printf("[AI_INIT] step3: get_inputs...\r\n");
        {
            stai_ptr in_real[1];
            stai_size n_in = 1;
            rc = stai_network_get_inputs(ai_net, in_real, &n_in);
            if (rc != STAI_SUCCESS) {
                printf("[AI_INIT] FAIL @ step3 (get_inputs), rc=%d\r\n", (int)rc);
                return;
            }
            ai_input_q_real = (int8_t*)in_real[0];
        }
        printf("[AI_INIT] step3 OK (in_buf=%p)\r\n", (void*)ai_input_q_real);

        printf("[AI_INIT] step4: get_outputs...\r\n");
        {
            stai_ptr out_real[1];
            stai_size n_out = 1;
            rc = stai_network_get_outputs(ai_net, out_real, &n_out);
            if (rc != STAI_SUCCESS) {
                printf("[AI_INIT] FAIL @ step4 (get_outputs), rc=%d\r\n", (int)rc);
                return;
            }
            ai_output_q_real = (int8_t*)out_real[0];
        }
        printf("[AI_INIT] step4 OK (out_buf=%p) -- AI ready\r\n", (void*)ai_output_q_real);

    ai_initialized = 1;
}

/* TinyML 4-B-1: dummy inference call + DWT cycle measurement.
 * Inputs are filled with constant 0.5f (dummy — not yet F1..F6).
 * Output is read but not used (KF/CSV unchanged).
 * Goal: confirm STAI runtime actually executes inference, and
 * measure cycles so we can verify the timing budget (RQ1). */
/* TinyML 4-B-2: real feature inference.
 * Inputs are the 6 features per thesis Table 3-1, sourced from
 * kf_cm and recent sensor state.  No normalisation yet (trained
 * model + per-feature min/max will be hardcoded later, thesis 4.4).
 * Output is read but not used (KF/CSV unchanged).
 *
 * Called every ToF measurement, just BEFORE kf_predict/kf_update,
 * so F1..F3 reflect the residual statistics from the most recent
 * KF update (Table 3-1 "KF Update 직전"). */
static void ai_run_real(float f1_residual,
                        float f2_residual_var,
                        float f3_residual_mean,
                        float f4_sensor_disagree,
                        float f5_meas_rate,
                        float f6_signal_rate)
{
    if (!ai_initialized) return;

    float feats_raw[6] = {
        f1_residual, f2_residual_var, f3_residual_mean,
        f4_sensor_disagree, f5_meas_rate, f6_signal_rate
    };

    /* (1) Standard normalization: (x - mean) / std  [thesis 표 3-3] */
    float feats_norm[6];
    for (int i = 0; i < 6; i++) {
        feats_norm[i] = (feats_raw[i] - AI_FEAT_MEAN[i]) / AI_FEAT_STD[i];
    }

    /* (2) Quantize float -> int8, write directly to PREALLOCATED input buf */
        for (int i = 0; i < 6; i++) {
            int32_t q = (int32_t)lroundf(feats_norm[i] / AI_IN_SCALE) + AI_IN_ZP;
            if (q < -128) q = -128;
            if (q >  127) q =  127;
            ai_input_q_real[i] = (int8_t)q;
        }

        /* (3) Inference + cycle measurement */
        uint32_t t0 = DWT->CYCCNT;
        stai_return_code rc = stai_network_run(ai_net, STAI_MODE_SYNC);
        uint32_t cycles = DWT->CYCCNT - t0;

        if (rc != STAI_SUCCESS) return;

        /* (4) Dequantize int8 -> float (log1p space), read from PREALLOCATED out buf */
        float r_log = ((int32_t)ai_output_q_real[0] - AI_OUT_ZP) * AI_OUT_SCALE;

    /* (5) expm1: log1p space -> R  [thesis 3.5.3]
     *     R = expm1(y) = e^y - 1
     *     Then clamp to [1, 10000] (same range as CM-AKF, thesis 3.4) */
    float r_pred = expm1f(r_log);
    if (r_pred < AI_R_MIN) r_pred = AI_R_MIN;
    if (r_pred > AI_R_MAX) r_pred = AI_R_MAX;

    ai_last_R_pred = r_pred;

    ai_infer_count++;
        ai_infer_cycles_sum += cycles;
        if (cycles > ai_infer_cycles_max) ai_infer_cycles_max = cycles;
        ai_last_infer_cycles = cycles;   /* E4: per-row CSV needs last, not max */
        ai_last_R = r_pred;
    }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_IWDG_Init();   /* IWDG starts here - subsequent code MUST refresh it */

  /* USER CODE BEGIN 2 */

  HAL_IWDG_Refresh(&hiwdg);
  DWT_Init();

  /* Boot delay: split 2s with refresh between (HC-06 boot + PuTTY connect) */
  Safe_Delay_ms(1000);
  Safe_Delay_ms(1000);

  printf("\r\n");
  HAL_IWDG_Refresh(&hiwdg);
  printf("========================================\r\n");
  printf(" Mode:      C (predict+update 1:1, synced to ToF DataReady)\r\n");
  printf(" KF:        Q=%.2f, R_INIT=%.1f, W=%d (Fixed + CM-AKF parallel)\r\n",
         KF_Q, KF_R_INIT, KF_WINDOW_SIZE);
  printf(" CSV:       25 fields, decimation=%lu (50Hz), DMA -> USART6\r\n",
         (unsigned long)LOG_DECIMATION);
  printf(" HC-SR04:   trigger %lums (~20Hz), echo TIM3 CH1 IC\r\n",
         (unsigned long)HCSR04_TRIG_PERIOD_MS);
  printf(" IWDG:      timeout=8.0s (dev), refresh every %lu loops (50ms)\r\n",
         (unsigned long)IWDG_REFRESH_EVERY);
  printf(" Budget:    %lu us / loop\r\n", (unsigned long)PHASE6_LOOP_BUDGET_US);
  printf("========================================\r\n");
  HAL_IWDG_Refresh(&hiwdg);

  Safe_Delay_ms(50);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  /* HC-SR04 trigger pin idle low */
  HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  /* HC-SR04 echo capture on TIM3 CH1 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

  HAL_IWDG_Refresh(&hiwdg);
  VL53L0X_Setup();
  HAL_IWDG_Refresh(&hiwdg);

  ai_init();

  /* Dual Kalman Filter: fixed (use_akf=false) and CM-AKF (use_akf=true) */
  KalmanFilter kf_fixed;
  KalmanFilter kf_cm;
  uint8_t      kf_initialised = 0;

  /* Wait for B1 before starting measurement (excludes stationary
   * transient before manual roll). IWDG is refreshed inside. */
  printf("[INIT] Setup done. LD2 blinking = press B1 to start.\r\n");
  HAL_IWDG_Refresh(&hiwdg);
  Wait_For_B1();
  HAL_IWDG_Refresh(&hiwdg);

  /* Drain any USART6 transfer still in flight from the boot banner
   * printf, so the header DMA below starts on an idle UART. */
  HAL_Delay(50);

  /* 25-column CSV header.
   * Sent via DMA on USART6 (the same path as the data rows) so it
   * never collides with printf's byte-by-byte blocking transmit.
   * We copy it into csv_tx_buf and wait for the DMA to finish before
   * starting the 200Hz loop, so the header is one clean line that
   * fully drains before any data row is queued. */
  {
	  int hn = snprintf(csv_tx_buf, CSV_BUF_SIZE,
	            "# CSV_HEADER: seq,timestamp_ms,tof_distance_mm,tof_signal_rate,"
	            "tof_range_status,us_distance_mm,encoder_distance_mm,"
	            "encoder_speed_mms,sensor_disagree,tof_meas_rate,gt_distance_mm,"
	            "scenario_id,"
	            "fixed_estimate_mm,fixed_residual,fixed_residual_var,"
	            "fixed_residual_mean,fixed_kalman_gain,fixed_innovation_cov,"
	            "cm_estimate_mm,cm_residual,cm_residual_var,cm_residual_mean,"
	            "cm_kalman_gain,cm_innovation_cov,cm_R,"
	            "tinyml_estimate_mm,tinyml_R,tinyml_infer_us\r\n");

      /* make sure no earlier USART6 transfer is still in flight */
      while (huart6.gState != HAL_UART_STATE_READY) { HAL_IWDG_Refresh(&hiwdg); }

      if (hn > 0 && hn < (int)CSV_BUF_SIZE) {
          HAL_UART_Transmit_DMA(&huart6, (uint8_t *)csv_tx_buf, (uint16_t)hn);
      }

      /* wait for the header DMA to fully drain */
      while (huart6.gState != HAL_UART_STATE_READY) { HAL_IWDG_Refresh(&hiwdg); }
  }
  HAL_IWDG_Refresh(&hiwdg);

  Safe_Delay_ms(50);   /* small margin before the data stream starts */
  HAL_TIM_Base_Start_IT(&htim6);

  uint32_t loop_count    = 0;
  uint32_t overrun_loop  = 0;
  uint64_t cycles_sum    = 0;
  uint32_t cycles_max    = 0;
  uint32_t cycles_min    = 0xFFFFFFFFU;

  uint32_t vl_data_ready_count = 0;
  uint32_t vl_read_ok_count    = 0;
  uint32_t vl_status0_count    = 0;
  uint32_t vl_i2c_err_count    = 0;
  uint16_t vl_last_dist_mm     = 0;
  uint8_t  vl_last_status      = 0xFF;
  float    vl_last_signal_mcps = 0.0f;
  float    vl_last_ambient_mcps = 0.0f;
  /* F5 Measurement Rate (thesis Table 3-1):
   * ratio of range_status==0 within a W=20 sliding window of ToF
   * measurements.  NOT the step-to-step ToF change-rate (the old
   * implementation).  W=20 matches the KF residual window so the
   * two statistics share the same time scale.
   * Implemented as a circular buffer of the last 20 range_status
   * values; F5 = (count of status==0) / (filled entries). */
  #define F5_WINDOW_SIZE   20U
  uint8_t  f5_status_buf[F5_WINDOW_SIZE] = {0};   /* 1 = status0, 0 = not */
  uint16_t f5_buf_idx          = 0;       /* next write position           */
  uint16_t f5_buf_count        = 0;       /* valid entries (max = W)       */
  uint16_t f5_status0_in_win   = 0;       /* running count of status0      */
  float    tof_meas_rate       = 0.0f;    /* CSV field: now = F5 ratio     */

  /* HC-SR04 */
  float    us_dist_mm          = 0.0f;
  uint32_t hcsr04_last_trig_ms = 0;

  int16_t  enc_l_prev = 0;
  int16_t  enc_r_prev = 0;
  int32_t  enc_l_total = 0;
  int32_t  enc_r_total = 0;

  /* [SCHEME C] Per-update accumulators.
   * enc_l_accum / enc_r_accum hold the pulse deltas summed since the
   * previous ToF update.  They are consumed (one predict) and reset
   * to zero each time a ToF measurement arrives.
   * last_update_ms timestamps the previous update so encoder_speed_mms
   * can be reported as the mean speed over the (variable) interval. */
  int32_t  enc_l_accum   = 0;
  int32_t  enc_r_accum   = 0;
  uint32_t last_update_ms = 0;
  uint8_t  have_last_update = 0;
  float    kf_u_mmps      = 0.0f;   /* last KF input, for CSV (mean speed)  */

  uint32_t kf_predict_count = 0;
  uint32_t kf_update_count  = 0;

  uint32_t csv_tx_attempts = 0;
  uint32_t csv_tx_drops    = 0;
  uint32_t csv_seq         = 0;
  uint32_t boot_ms         = HAL_GetTick();

  uint32_t iwdg_refresh_count = 0;

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (loop_count < PHASE6_N_TEST_LOOPS)
  {
      if (!loop_flag) {
          continue;
      }
      loop_flag = 0;

      if ((loop_count % IWDG_REFRESH_EVERY) == 0) {
          HAL_IWDG_Refresh(&hiwdg);
          iwdg_refresh_count++;
      }

      uint32_t t_start = DWT->CYCCNT;

      /* ---- HC-SR04 trigger (every HCSR04_TRIG_PERIOD_MS) ---- */
      uint32_t now_ms = HAL_GetTick();
      if ((now_ms - hcsr04_last_trig_ms) >= HCSR04_TRIG_PERIOD_MS) {
          hcsr04_last_trig_ms = now_ms;
          HCSR04_Trigger();
      }
      /* ---- HC-SR04 echo result ready? ---- */
      if (echo_ready) {
          echo_ready = 0;
          us_dist_mm = (float)echo_pulse_us * HCSR04_US_TO_MM;
      }

      /* ---- Encoder read (R-side sign corrected) ----
       * [SCHEME C] The KF is NOT stepped here.  We only read the
       * encoder counters and ACCUMULATE the pulse deltas.  enc_l_total
       * / enc_r_total still track the absolute pulse count for the
       * encoder_distance_mm CSV field; enc_l_accum / enc_r_accum hold
       * the deltas waiting to be fed to the next kf_predict. */
      int16_t enc_l_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
      int16_t enc_r_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
      int16_t dl = enc_l_now - enc_l_prev;
      int16_t dr = -(enc_r_now - enc_r_prev);   /* R motor wiring inverted */
      enc_l_prev = enc_l_now;
      enc_r_prev = enc_r_now;
      enc_l_total += dl;
      enc_r_total += dr;
      enc_l_accum += dl;
      enc_r_accum += dr;

      /* ---- VL53L0X read ---- */
      uint8_t data_ready = 0;
      VL53L0X_Error vl_st = VL53L0X_GetMeasurementDataReady(pVL53L0X, &data_ready);
      if (vl_st != VL53L0X_ERROR_NONE) {
          vl_i2c_err_count++;
      } else if (data_ready) {
          vl_data_ready_count++;
          VL53L0X_RangingMeasurementData_t m;
          vl_st = VL53L0X_GetRangingMeasurementData(pVL53L0X, &m);
          if (vl_st != VL53L0X_ERROR_NONE) {
              vl_i2c_err_count++;
          } else {
              vl_read_ok_count++;
              vl_last_dist_mm      = m.RangeMilliMeter;
              vl_last_status       = m.RangeStatus;
              vl_last_signal_mcps  = (float)m.SignalRateRtnMegaCps  / 65536.0f;
              vl_last_ambient_mcps = (float)m.AmbientRateRtnMegaCps / 65536.0f;

              /* ---- F5 Measurement Rate: status==0 ratio over W=20 ----
               * Every ToF measurement (status 0 or not) is pushed into
               * the circular buffer.  f5_status0_in_win is kept as a
               * running count so the ratio is O(1).  This is the thesis
               * Table 3-1 definition; the old change-rate code is gone. */
              {
                  uint8_t is_status0 = (m.RangeStatus == 0) ? 1U : 0U;
                  if (f5_buf_count >= F5_WINDOW_SIZE) {
                      /* buffer full: subtract the entry being overwritten */
                      f5_status0_in_win -= f5_status_buf[f5_buf_idx];
                  }
                  f5_status_buf[f5_buf_idx] = is_status0;
                  f5_status0_in_win += is_status0;
                  f5_buf_idx = (f5_buf_idx + 1U) % F5_WINDOW_SIZE;
                  if (f5_buf_count < F5_WINDOW_SIZE) {
                      f5_buf_count++;
                  }
                  tof_meas_rate = (f5_buf_count > 0U)
                      ? ((float)f5_status0_in_win / (float)f5_buf_count)
                      : 0.0f;
              }

              if (m.RangeStatus == 0) {
                  vl_status0_count++;

                  /* ---- [SCHEME C] KF step: exactly one predict +
                   *      one update, fed the displacement accumulated
                   *      since the previous ToF update. ----
                   *
                   * KF state x is "distance to wall".  Moving forward
                   * DECREASES x, so the encoder input is negated.
                   * The whole accumulated displacement goes into a
                   * single predict, so predict:update is strictly
                   * 1:1 and phase-aligned regardless of how many
                   * 200Hz loops (or how much ToF jitter) elapsed.
                   *
                   * accum_pulses (signed) -> KF input:
                   *   D_mm = accum_pulses * 0.5 * MM_PER_PULSE
                   *   u    = D_mm / KF_B   (PULSES_TO_KF_U folds the
                   *          MM_PER_PULSE / KF_B constant)
                   */
                  float accum_pulses =
                      ((float)enc_l_accum + (float)enc_r_accum) * 0.5f;
                  float u_kf = accum_pulses * PULSES_TO_KF_U;

                  /* mean speed over the interval, for the CSV field
                   * (positive = forward, per the thesis GT convention) */
                  uint32_t now_upd_ms = HAL_GetTick();
                  if (have_last_update) {
                      uint32_t dt_ms = now_upd_ms - last_update_ms;
                      float d_mm = accum_pulses * MM_PER_PULSE;
                      kf_u_mmps = (dt_ms > 0U)
                          ? (d_mm * 1000.0f / (float)dt_ms)
                          : 0.0f;
                  } else {
                      kf_u_mmps = 0.0f;
                  }
                  last_update_ms   = now_upd_ms;
                  have_last_update = 1;

                  if (!kf_initialised) {
                                        kf_init(&kf_fixed,  (float)m.RangeMilliMeter,
                                                KF_R_INIT, KF_R_INIT);
                                        kf_init(&kf_cm,     (float)m.RangeMilliMeter,
                                                KF_R_INIT, KF_R_INIT);
                                        kf_init(&kf_tinyml, (float)m.RangeMilliMeter,
                                                KF_R_INIT, KF_R_INIT);
                                        kf_initialised = 1;
                  } else {
                                        /* TinyML 4-C-2: run inference BEFORE KF predict/update,
                                         * so F1..F3 (residual stats) come from the previous
                                         * update — matches thesis Table 3-1 "KF Update 직전".
                                         * Note: kf_get_residual_stats signature is
                                         *       (kf, *mean, *var) — mean first, var second. */
                                        float ai_f2_var = 0.0f, ai_f3_mean = 0.0f;
                                        kf_get_residual_stats(&kf_cm, &ai_f3_mean, &ai_f2_var);
                                        float ai_f4_disagree = fabsf((float)vl_last_dist_mm - us_dist_mm);

                                        ai_run_real(kf_cm.residual,       /* F1 */
                                                    ai_f2_var,            /* F2 */
                                                    ai_f3_mean,           /* F3 */
                                                    ai_f4_disagree,       /* F4 */
                                                    tof_meas_rate,        /* F5 */
                                                    vl_last_signal_mcps); /* F6 */

                                        /* Inject TinyML-predicted R into kf_tinyml.
                                         * kf_update with use_akf=false uses kf->R as-is,
                                         * so we set it here before calling update. */
                                        kf_tinyml.R = ai_last_R_pred;

                                        /* one predict (whole accumulated displacement) */
                                        kf_predict(&kf_fixed,  -u_kf);
                                        kf_predict(&kf_cm,     -u_kf);
                                        kf_predict(&kf_tinyml, -u_kf);
                                        kf_predict_count++;
                                        /* one update */
                                        kf_update(&kf_fixed,  (float)m.RangeMilliMeter, false);
                                        kf_update(&kf_cm,     (float)m.RangeMilliMeter, true);
                                        kf_update(&kf_tinyml, (float)m.RangeMilliMeter, false);
                                        kf_update_count++;
                                    }

                  /* accumulator consumed -> reset for next interval */
                  enc_l_accum = 0;
                  enc_r_accum = 0;
              }
          }
          VL53L0X_ClearInterruptMask(pVL53L0X, 0);
      }

      /* ---- CSV logging (decimated to 50Hz) ---- */
      if (kf_initialised && ((loop_count % LOG_DECIMATION) == 0)) {
          csv_tx_attempts++;

          if (huart6.gState != HAL_UART_STATE_READY) {
              csv_tx_drops++;
          } else {
              float fx_mean = 0.0f, fx_var = 0.0f;
              float cm_mean = 0.0f, cm_var = 0.0f;
              kf_get_residual_stats(&kf_fixed, &fx_mean, &fx_var);
              kf_get_residual_stats(&kf_cm,    &cm_mean, &cm_var);

              float enc_dist_mm =
                  ((float)enc_l_total + (float)enc_r_total) * 0.5f * MM_PER_PULSE;
              float sensor_disagree = fabsf((float)vl_last_dist_mm - us_dist_mm);
              uint32_t ts_ms = HAL_GetTick() - boot_ms;

              /* tinyml infer time (us) — per-row LAST inference time.
                             * For E4 30-min latency time-series analysis: each CSV row
                             * must reflect the most recent inference cycles, not the
                             * running max.  ai_last_infer_cycles is updated inside
                             * ai_run_real() (see below). */
                            uint32_t hclk_mhz_csv = HAL_RCC_GetHCLKFreq() / 1000000U;
                            uint32_t tinyml_last_us = (ai_infer_count > 0U)
                                ? (ai_last_infer_cycles / hclk_mhz_csv)
                                : 0U;

                            int n = snprintf(csv_tx_buf, CSV_BUF_SIZE,
                                /* shared 12 */
                                "%lu,%lu,%u,%.3f,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,"
                                /* fixed 6 */
                                "%.3f,%.3f,%.3f,%.3f,%.6f,%.3f,"
                                /* cm 7 */
                                "%.3f,%.3f,%.3f,%.3f,%.6f,%.3f,%.3f,"
                                /* tinyml 3 */
                                "%.3f,%.3f,%lu\r\n",
                                /* --- shared --- */
                                (unsigned long)csv_seq,
                                (unsigned long)ts_ms,
                                vl_last_dist_mm,
                                vl_last_signal_mcps,
                                vl_last_status,
                                us_dist_mm,
                                enc_dist_mm,
                                kf_u_mmps,                    /* mean speed over interval    */
                                sensor_disagree,
                                tof_meas_rate,
                                0.0f,                         /* gt_distance_mm: post-fill   */
                                (unsigned)CSV_SCENARIO_ID,
                                /* --- fixed --- */
                                kf_fixed.x, kf_fixed.residual, fx_var, fx_mean,
                                kf_fixed.K, kf_fixed.S,
                                /* --- cm --- */
                                kf_cm.x, kf_cm.residual, cm_var, cm_mean,
                                kf_cm.K, kf_cm.S, kf_cm.R,
                                /* --- tinyml --- */
                                kf_tinyml.x, kf_tinyml.R, (unsigned long)tinyml_last_us);

              if (n > 0 && n < (int)CSV_BUF_SIZE) {
                  HAL_UART_Transmit_DMA(&huart6, (uint8_t *)csv_tx_buf, (uint16_t)n);
                  csv_seq++;
              }
          }
      }

      uint32_t t_end = DWT->CYCCNT;

      uint32_t cycles = t_end - t_start;
      cycles_sum += cycles;
      if (cycles > cycles_max) cycles_max = cycles;
      if (cycles < cycles_min) cycles_min = cycles;

      uint32_t loop_us = cycles / (HAL_RCC_GetHCLKFreq() / 1000000U);
      if (loop_us > PHASE6_LOOP_BUDGET_US) overrun_loop++;

      loop_count++;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

  HAL_TIM_Base_Stop_IT(&htim6);
  HAL_IWDG_Refresh(&hiwdg);

  /* Wait for any pending DMA TX to finish */
  uint32_t drain_start = HAL_GetTick();
  while (huart6.gState != HAL_UART_STATE_READY) {
      if ((HAL_GetTick() - drain_start) > 100U) {
          HAL_IWDG_Refresh(&hiwdg);
          drain_start = HAL_GetTick();
      }
  }
  HAL_IWDG_Refresh(&hiwdg);

  VL53L0X_StopMeasurement(pVL53L0X);
  HAL_IWDG_Refresh(&hiwdg);

  uint32_t isr_overrun_final = isr_overrun_count;
  uint32_t total_ticks       = loop_tick;

  uint32_t hclk_mhz   = HAL_RCC_GetHCLKFreq() / 1000000U;
  uint32_t mean_ns    = (uint32_t)(((cycles_sum * 1000U) / loop_count) / hclk_mhz);
  uint32_t max_ns     = (uint32_t)(((uint64_t)cycles_max * 1000U) / hclk_mhz);
  uint32_t min_ns     = (uint32_t)(((uint64_t)cycles_min * 1000U) / hclk_mhz);
  uint32_t mean_us    = mean_ns / 1000U;
  uint32_t max_us     = max_ns  / 1000U;
  uint32_t mean_ns_r  = mean_ns % 1000U;
  uint32_t max_ns_r   = max_ns  % 1000U;

  HAL_IWDG_Refresh(&hiwdg);

  HAL_IWDG_Refresh(&hiwdg);
  printf("# Loops:                  %lu\r\n", (unsigned long)loop_count);
  printf("# TIM6 ticks:             %lu\r\n", (unsigned long)total_ticks);
  printf("# Mean body:              %lu.%03lu us\r\n",
         (unsigned long)mean_us, (unsigned long)mean_ns_r);
  printf("# Min body:               %lu ns\r\n", (unsigned long)min_ns);
  printf("# Max body:               %lu.%03lu us\r\n",
         (unsigned long)max_us, (unsigned long)max_ns_r);
  HAL_IWDG_Refresh(&hiwdg);
  printf("# Body overrun (>%luus): %lu\r\n",
         (unsigned long)PHASE6_LOOP_BUDGET_US, (unsigned long)overrun_loop);
  printf("# ISR overrun:            %lu\r\n", (unsigned long)isr_overrun_final);
  printf("# VL53L0X DataReady:      %lu (~%lu Hz)\r\n",
         (unsigned long)vl_data_ready_count,
         (unsigned long)(vl_data_ready_count * 200U / loop_count));
  printf("# VL53L0X Read OK:        %lu (status0: %lu)\r\n",
         (unsigned long)vl_read_ok_count, (unsigned long)vl_status0_count);
  printf("# I2C errors:             %lu\r\n", (unsigned long)vl_i2c_err_count);
  HAL_IWDG_Refresh(&hiwdg);
  printf("# KF predict / update:    %lu / %lu  (Scheme C: must be equal)\r\n",
         (unsigned long)kf_predict_count, (unsigned long)kf_update_count);
  if (kf_initialised) {
      printf("# Fixed final: x=%.3f P=%.3f R=%.3f K=%.6f\r\n",
             kf_fixed.x, kf_fixed.P, kf_fixed.R, kf_fixed.K);
      printf("# CM-AKF final: x=%.3f P=%.3f R=%.3f K=%.6f\r\n",
             kf_cm.x, kf_cm.P, kf_cm.R, kf_cm.K);
  }
  printf("# CSV TX:    attempts=%lu  drops=%lu  seq=%lu\r\n",
         (unsigned long)csv_tx_attempts,
         (unsigned long)csv_tx_drops,
         (unsigned long)csv_seq);
  if (ai_infer_count > 0) {
      uint32_t hclk_mhz = HAL_RCC_GetHCLKFreq() / 1000000U;
      uint32_t mean_ns = (uint32_t)(((uint64_t)ai_infer_cycles_sum * 1000U) / ai_infer_count) / hclk_mhz;
      uint32_t max_ns  = (uint32_t)((uint64_t)ai_infer_cycles_max * 1000U) / hclk_mhz;
      printf("# TinyML infer: count=%lu mean=%lu.%03lu us max=%lu.%03lu us last_R=%.4f\r\n",
             (unsigned long)ai_infer_count,
             (unsigned long)(mean_ns / 1000U), (unsigned long)(mean_ns % 1000U),
             (unsigned long)(max_ns  / 1000U), (unsigned long)(max_ns  % 1000U),
             ai_last_R);
  }
  printf("# IWDG:      refreshes=%lu (every %lu loops, timeout 8s)\r\n",
         (unsigned long)iwdg_refresh_count, (unsigned long)IWDG_REFRESH_EVERY);
  HAL_IWDG_Refresh(&hiwdg);
  printf("# ==================================\r\n");
  HAL_IWDG_Refresh(&hiwdg);

  /* Idle - heartbeat LED, keep refreshing IWDG so the demo doesn't reset */
  while (1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_IWDG_Refresh(&hiwdg);
      HAL_Delay(500);
  }
}

/* ============================================================
 * Wait_For_B1: block until the B1 (PC13) button is pressed.
 * NUCLEO B1 reads HIGH when released, LOW when pressed.
 * IWDG is refreshed inside the loop so a long wait (the operator
 * getting ready to roll the robot) does not trigger a reset.
 *
 * The LD2 LED blinks while waiting -> this is the operator's
 * "press B1 now" cue (the logger hides pre-header banner lines,
 * so the prompt is not visible on the PC).  After B1 is pressed
 * the LED is left ON to indicate "measuring".
 * ============================================================ */
static void Wait_For_B1(void)
{
    /* Wait while released (HIGH) — blink LD2 as a visible cue */
    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(100);   /* ~5 Hz blink */
    }
    /* Simple debounce */
    HAL_Delay(20);
    HAL_IWDG_Refresh(&hiwdg);

    /* B1 pressed — LED solid ON = "measuring" */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

/* ============================================================
 * HCSR04_Trigger: fire a 10us HIGH pulse on the trigger pin.
 * echo_capture_state is reset first so a missed echo from the
 * previous cycle does not desync rising/falling capture.
 * ============================================================ */
static void HCSR04_Trigger(void)
{
    echo_capture_state = 0;   /* recover from a missed echo */
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,
                                  TIM_INPUTCHANNELPOLARITY_RISING);

    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    DWT_Delay_us(HCSR04_TRIG_PULSE_US);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}

/* ============================================================
 * Safe_Delay_ms: HAL_Delay that periodically refreshes IWDG.
 * Granularity: 100ms refresh interval, well below 8s timeout.
 * ============================================================ */
static void Safe_Delay_ms(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        HAL_IWDG_Refresh(&hiwdg);
        if (ms - (HAL_GetTick() - start) > 100U) {
            HAL_Delay(100);
        } else {
            HAL_Delay(ms - (HAL_GetTick() - start));
        }
    }
    HAL_IWDG_Refresh(&hiwdg);
}

/* ============================================================
 * VL53L0X setup with IWDG refresh between long steps
 * ============================================================ */
static void VL53L0X_Setup(void)
{
    VL53L0X_Error vl_status = VL53L0X_ERROR_NONE;

    printf("[INIT] VL53L0X starting...\r\n");
    HAL_IWDG_Refresh(&hiwdg);

    pVL53L0X->I2cHandle  = &hi2c1;
    pVL53L0X->I2cDevAddr = 0x52;

    vl_status = VL53L0X_ResetDevice(pVL53L0X);
    HAL_IWDG_Refresh(&hiwdg);
    Safe_Delay_ms(50);

    vl_status = VL53L0X_DataInit(pVL53L0X);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("DataInit FAIL\r\n"); Error_Handler(); }
    HAL_IWDG_Refresh(&hiwdg);

    vl_status = VL53L0X_StaticInit(pVL53L0X);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("StaticInit FAIL\r\n"); Error_Handler(); }
    HAL_IWDG_Refresh(&hiwdg);

    uint8_t VhvSettings = 0, PhaseCal = 0;
    vl_status = VL53L0X_PerformRefCalibration(pVL53L0X, &VhvSettings, &PhaseCal);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("RefCal FAIL\r\n"); Error_Handler(); }
    HAL_IWDG_Refresh(&hiwdg);

    uint32_t refSpadCount = 0;
    uint8_t  isApertureSpads = 0;
    vl_status = VL53L0X_PerformRefSpadManagement(pVL53L0X, &refSpadCount, &isApertureSpads);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("RefSpad FAIL\r\n"); Error_Handler(); }
    HAL_IWDG_Refresh(&hiwdg);

    vl_status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pVL53L0X, VL53L0X_TIMING_BUDGET_US);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("TimingBudget FAIL\r\n"); Error_Handler(); }

    vl_status = VL53L0X_SetDeviceMode(pVL53L0X, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("DeviceMode FAIL\r\n"); Error_Handler(); }

    vl_status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(pVL53L0X, VL53L0X_INTER_PERIOD_MS);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("InterPeriod FAIL\r\n"); Error_Handler(); }

    vl_status = VL53L0X_StartMeasurement(pVL53L0X);
    if (vl_status != VL53L0X_ERROR_NONE) { printf("StartMeasurement FAIL\r\n"); Error_Handler(); }

    HAL_IWDG_Refresh(&hiwdg);
    printf("[INIT] VL53L0X ready (continuous, 50Hz).\r\n");
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief IWDG Initialization Function
  *        Initial Reload = 1000 (2s) - immediately overridden below to 4000 (8s)
  *        Why override here instead of in CubeMX: avoids .ioc re-generation
  *        for a one-line dev-mode change. For production, set in CubeMX directly.
  */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload    = IWDG_RELOAD_VAL;   /* 4000 -> 8.0s timeout */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM4_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART6_UART_Init(void)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK) { Error_Handler(); }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetTick_us(void)
{
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
}

void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        if (loop_flag) {
            isr_overrun_count++;
        }
        loop_flag = 1;
        loop_tick++;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (echo_capture_state == 0)
        {
            echo_rising_tick = cap;
            echo_capture_state = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            echo_falling_tick = cap;
            if (echo_falling_tick >= echo_rising_tick)
                echo_pulse_us = echo_falling_tick - echo_rising_tick;
            else
                echo_pulse_us = (0xFFFF - echo_rising_tick) + echo_falling_tick + 1;
            echo_capture_state = 0;
            echo_ready = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
