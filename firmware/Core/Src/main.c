/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Phase 6 Step 6 (v2): IWDG margin enlarged, init hardened
  *
  * Changes from Step 6 v1:
  *   - IWDG override after MX_IWDG_Init(): Reload 1000 -> 4000 (timeout 8s)
  *   - Boot delay split: 2x500ms with IWDG refresh between
  *   - IWDG refreshes every printf chunk during init/teardown
  *   - All HAL_Delay() calls preceded by IWDG refresh
  *
  * Why 8s timeout for development:
  *   - 2s was too tight (matched HAL_Delay 2000 exactly -> race)
  *   - 8s allows debugger breaks of up to 8s without triggering reset
  *   - For final production code, can be reduced after stability proven
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PHASE6_N_TEST_LOOPS     1000U
#define PHASE6_LOOP_BUDGET_US   4500U

#define VL53L0X_TIMING_BUDGET_US    20000U
#define VL53L0X_INTER_PERIOD_MS     20U

#define MM_PER_PULSE            0.05397f
#define LOOP_DT_SEC             0.005f
#define PULSES_TO_MMPS          (MM_PER_PULSE / LOOP_DT_SEC)

#define LOG_DECIMATION          4U
#define CSV_BUF_SIZE            256U
#define CSV_SCENARIO_ID         0U

#define IWDG_REFRESH_EVERY      10U     /* refresh every 10 loops = 50ms */
#define IWDG_RELOAD_VAL         4000U   /* 4000*64/32000 = 8.0s timeout  */

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  printf(" Phase 6 - Step 6 v2: KF + CSV + IWDG\r\n");
  printf(" Target:    N=%lu loops (~5s @ 200Hz)\r\n",
         (unsigned long)PHASE6_N_TEST_LOOPS);
  printf(" Mode:      A (predict 200Hz, update on DataReady)\r\n");
  printf(" KF:        Q=%.2f, R_INIT=%.1f, W=%d (Fixed KF, no AKF)\r\n",
         KF_Q, KF_R_INIT, KF_WINDOW_SIZE);
  printf(" CSV:       18 fields, decimation=%lu (50Hz), DMA -> USART6\r\n",
         (unsigned long)LOG_DECIMATION);
  printf(" IWDG:      timeout=8.0s (dev), refresh every %lu loops (50ms)\r\n",
         (unsigned long)IWDG_REFRESH_EVERY);
  printf(" Budget:    %lu us / loop\r\n", (unsigned long)PHASE6_LOOP_BUDGET_US);
  printf("========================================\r\n");
  HAL_IWDG_Refresh(&hiwdg);

  printf("# CSV_HEADER: seq,timestamp_ms,enc_L,enc_R,pos_L_mm,pos_R_mm,"
         "tof_dist_mm,tof_status,tof_signal_mcps,tof_ambient_mcps,"
         "kf_estimate,kf_covariance,residual,residual_var,residual_mean,"
         "kalman_gain,innovation_cov,scenario_id\r\n");
  HAL_IWDG_Refresh(&hiwdg);

  Safe_Delay_ms(50);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  HAL_IWDG_Refresh(&hiwdg);
  VL53L0X_Setup();
  HAL_IWDG_Refresh(&hiwdg);

  KalmanFilter kf;
  uint8_t      kf_initialised = 0;

  printf("[INIT] Starting TIM6 @ 200Hz. CSV stream begins.\r\n");
  HAL_IWDG_Refresh(&hiwdg);
  Safe_Delay_ms(50);
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

  int16_t  enc_l_prev = 0;
  int16_t  enc_r_prev = 0;
  int32_t  enc_l_total = 0;
  int32_t  enc_r_total = 0;

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

      int16_t enc_l_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
      int16_t enc_r_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
      int16_t dl = enc_l_now - enc_l_prev;
      int16_t dr = enc_r_now - enc_r_prev;
      enc_l_prev = enc_l_now;
      enc_r_prev = enc_r_now;
      enc_l_total += dl;
      enc_r_total += dr;
      float avg_dpulses = ((float)dl + (float)dr) * 0.5f;
      float u_mmps      = avg_dpulses * PULSES_TO_MMPS;

      if (kf_initialised) {
          kf_predict(&kf, u_mmps);
          kf_predict_count++;
      }

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
              if (m.RangeStatus == 0) {
                  vl_status0_count++;
                  if (!kf_initialised) {
                      kf_init(&kf, (float)m.RangeMilliMeter, KF_R_INIT, KF_R_INIT);
                      kf_initialised = 1;
                  } else {
                      kf_update(&kf, (float)m.RangeMilliMeter, false);
                      kf_update_count++;
                  }
              }
          }
          VL53L0X_ClearInterruptMask(pVL53L0X, 0);
      }

      if (kf_initialised && ((loop_count % LOG_DECIMATION) == 0)) {
          csv_tx_attempts++;

          if (huart6.gState != HAL_UART_STATE_READY) {
              csv_tx_drops++;
          } else {
              float r_mean = 0.0f, r_var = 0.0f;
              kf_get_residual_stats(&kf, &r_mean, &r_var);

              float pos_l_mm = (float)enc_l_total * MM_PER_PULSE;
              float pos_r_mm = (float)enc_r_total * MM_PER_PULSE;
              uint32_t ts_ms = HAL_GetTick() - boot_ms;

              int n = snprintf(csv_tx_buf, CSV_BUF_SIZE,
                  "%lu,%lu,%ld,%ld,%.3f,%.3f,"
                  "%u,%u,%.3f,%.3f,"
                  "%.3f,%.3f,%.3f,%.3f,%.3f,"
                  "%.6f,%.3f,%u\r\n",
                  (unsigned long)csv_seq,
                  (unsigned long)ts_ms,
                  (long)enc_l_total, (long)enc_r_total,
                  pos_l_mm, pos_r_mm,
                  vl_last_dist_mm, vl_last_status,
                  vl_last_signal_mcps, vl_last_ambient_mcps,
                  kf.x, kf.P, kf.residual, r_var, r_mean,
                  kf.K, kf.S,
                  (unsigned)CSV_SCENARIO_ID);

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

  printf("\r\n# ===== Phase 6 Step 6 Results =====\r\n");
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
  printf("# KF predict / update:    %lu / %lu\r\n",
         (unsigned long)kf_predict_count, (unsigned long)kf_update_count);
  if (kf_initialised) {
      printf("# KF final:  x=%.3f P=%.3f R=%.3f K=%.6f\r\n",
             kf.x, kf.P, kf.R, kf.K);
  }
  printf("# CSV TX:    attempts=%lu  drops=%lu  seq=%lu\r\n",
         (unsigned long)csv_tx_attempts,
         (unsigned long)csv_tx_drops,
         (unsigned long)csv_seq);
  printf("# IWDG:      refreshes=%lu (every %lu loops, timeout 8s)\r\n",
         (unsigned long)iwdg_refresh_count, (unsigned long)IWDG_REFRESH_EVERY);
  HAL_IWDG_Refresh(&hiwdg);

  int pass = 1;
  if (overrun_loop > 0)       { pass = 0; printf("# FAIL: body overrun\r\n"); }
  if (isr_overrun_final > 0)  { pass = 0; printf("# FAIL: ISR overrun\r\n"); }
  if (vl_i2c_err_count > 0)   { pass = 0; printf("# FAIL: I2C errors\r\n"); }
  if (csv_tx_drops > 0)       { pass = 0; printf("# FAIL: CSV TX drops\r\n"); }
  if (!kf_initialised)        { pass = 0; printf("# FAIL: KF not init\r\n"); }
  if (kf_update_count < 10)   { pass = 0; printf("# FAIL: too few KF updates\r\n"); }
  if (pass) {
      printf("# RESULT: PASS - KF + CSV + IWDG integrated\r\n");
  } else {
      printf("# RESULT: FAIL\r\n");
  }
  printf("# ==================================\r\n");
  printf("# To verify IWDG trigger: pull I2C jumper, wait ~8s, expect header reprint.\r\n");
  HAL_IWDG_Refresh(&hiwdg);

  /* Idle - heartbeat LED, keep refreshing IWDG so the demo doesn't reset */
  while (1) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_IWDG_Refresh(&hiwdg);
      HAL_Delay(500);
  }
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
