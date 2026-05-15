/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Phase 4-B: Motor + Sensor Noise Measurement
  *
  * Purpose:
  *   Measure VL53L0X & HC-SR04 noise while both motors run at 50% PWM.
  *   Quantitative justification for manual-rolling decision in Phase 7.
  *
  * Trigger:  Press B1 (PC13) to start measurement.
  * Output:   HC-06 Bluetooth (USART6, 115200 baud, PuTTY).
  * Baseline: Phase 1 VL53L0X @ 100mm static -> sigma=1.53mm, I2C err=0.
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

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PHASE4B_N_SAMPLES       100     /* VL53L0X measurement count */
#define PHASE4B_PERIOD_MS       20      /* 50Hz target */
#define PHASE4B_HCSR04_EVERY    10      /* HC-SR04 trigger every Nth loop */

#define MOTOR_PWM_DUTY          32768   /* 50% of TIM1 ARR=65535 */
#define MOTOR_STARTUP_WAIT_MS   1000    /* steady-state wait after motor ON */

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
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
/* USER CODE BEGIN PFP */

void  DWT_Init(void);
void  DWT_Delay_us(uint32_t us);
uint32_t DWT_GetTick_us(void);
void  HCSR04_Trigger(void);
int   __io_putchar(int ch);

static void Motor_Start_50pct(void);
static void Motor_Stop(void);
static void Wait_For_B1_Press(void);

/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

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

  /* USER CODE BEGIN 2 */

  DWT_Init();

  HAL_Delay(2000);  /* HC-06 boot + PuTTY connect window */

  printf("\r\n");
  printf("========================================\r\n");
  printf(" Phase 4-B: Motor + Sensor Noise Test\r\n");
  printf(" Target:  VL53L0X @ 100mm, N=%d samples\r\n", PHASE4B_N_SAMPLES);
  printf(" Motor:   50%% PWM, both forward direction\r\n");
  printf(" Channel: HC-06 (USART6 @ 115200)\r\n");
  printf("========================================\r\n");

  /* ---------------- VL53L0X initialization ---------------- */
  printf("\r\n[INIT] VL53L0X starting up...\r\n");

  pVL53L0X->I2cHandle  = &hi2c1;
  pVL53L0X->I2cDevAddr = 0x52;

  VL53L0X_Error vl_status = VL53L0X_ERROR_NONE;

  printf("       ResetDevice...               ");
  vl_status = VL53L0X_ResetDevice(pVL53L0X);
  HAL_Delay(50);
  printf("%s\r\n", (vl_status == VL53L0X_ERROR_NONE) ? "OK" : "WARN (continuing)");

  printf("       DataInit...                  ");
  vl_status = VL53L0X_DataInit(pVL53L0X);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("       StaticInit...                ");
  vl_status = VL53L0X_StaticInit(pVL53L0X);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  uint8_t VhvSettings = 0, PhaseCal = 0;
  printf("       PerformRefCalibration...     ");
  vl_status = VL53L0X_PerformRefCalibration(pVL53L0X, &VhvSettings, &PhaseCal);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  uint32_t refSpadCount = 0;
  uint8_t  isApertureSpads = 0;
  printf("       PerformRefSpadManagement...  ");
  vl_status = VL53L0X_PerformRefSpadManagement(pVL53L0X, &refSpadCount, &isApertureSpads);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("       SetTimingBudget 20ms...      ");
  vl_status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pVL53L0X, 20000);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("       SetDeviceMode CONTINUOUS...  ");
  vl_status = VL53L0X_SetDeviceMode(pVL53L0X, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("       SetInterMeasurementPeriod... ");
  vl_status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(pVL53L0X, PHASE4B_PERIOD_MS);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("[INIT] VL53L0X ready.\r\n");

  /* ---------------- TB6612FNG idle state ---------------- */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);  /* STBY HIGH */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  printf("[INIT] TB6612FNG STBY=HIGH, PWM=0 (idle).\r\n");

  /* ---------------- Wait for B1 trigger ---------------- */
  printf("\r\n>>> Place white target @ 100mm.\r\n");
  printf(">>> Press B1 (blue button) to start measurement.\r\n");
  Wait_For_B1_Press();

  /* ---------------- Motor ON + steady-state wait ---------------- */
  printf("\r\n[TEST] Motor START (both forward, 50%% PWM)\r\n");
  Motor_Start_50pct();

  printf("[TEST] Steady-state wait %d ms...\r\n", MOTOR_STARTUP_WAIT_MS);
  HAL_Delay(MOTOR_STARTUP_WAIT_MS);

  /* ---------------- Start VL53L0X continuous ranging ---------------- */
  vl_status = VL53L0X_StartMeasurement(pVL53L0X);
  if (vl_status != VL53L0X_ERROR_NONE) {
      printf("[ABORT] VL53L0X_StartMeasurement failed (err=%d)\r\n", vl_status);
      Motor_Stop();
      Error_Handler();
  }

  /* ---------------- Measurement loop ---------------- */
  printf("\r\n[TEST] Measurement START (N=%d)\r\n", PHASE4B_N_SAMPLES);
  printf("       Format: idx, vl_dist_mm, vl_status, vl_signal_MCps, sr04_us\r\n");

  uint32_t vl_status0_count = 0;
  uint32_t vl_i2c_err_count = 0;
  double   vl_sum = 0.0, vl_sum_sq = 0.0;
  uint16_t vl_min = 0xFFFF, vl_max = 0;

  uint32_t sr04_count = 0;
  uint32_t sr04_err_count = 0;
  double   sr04_sum_us = 0.0, sr04_sum_sq_us = 0.0;
  uint32_t sr04_min_us = 0xFFFFFFFF, sr04_max_us = 0;

  uint32_t samples_collected = 0;
  uint32_t poll_timeout_count = 0;

  while (samples_collected < PHASE4B_N_SAMPLES)
  {
      /* Wait for VL53L0X data ready (poll, 100ms timeout) */
      uint8_t  data_ready = 0;
      uint32_t poll_start = HAL_GetTick();
      while (!data_ready && (HAL_GetTick() - poll_start) < 100)
      {
          vl_status = VL53L0X_GetMeasurementDataReady(pVL53L0X, &data_ready);
          if (vl_status != VL53L0X_ERROR_NONE) {
              vl_i2c_err_count++;
              break;
          }
      }

      if (!data_ready) {
          poll_timeout_count++;
          continue;
      }

      /* Read measurement */
      VL53L0X_RangingMeasurementData_t m;
      vl_status = VL53L0X_GetRangingMeasurementData(pVL53L0X, &m);
      if (vl_status != VL53L0X_ERROR_NONE) {
          vl_i2c_err_count++;
          VL53L0X_ClearInterruptMask(pVL53L0X, 0);
          continue;
      }

      samples_collected++;

      float signal_mcps = (float)m.SignalRateRtnMegaCps / 65536.0f;

      if (m.RangeStatus == 0) {
          uint16_t d = m.RangeMilliMeter;
          vl_status0_count++;
          vl_sum    += (double)d;
          vl_sum_sq += (double)d * (double)d;
          if (d < vl_min) vl_min = d;
          if (d > vl_max) vl_max = d;
      }

      /* HC-SR04 every Nth sample */
      uint32_t sr04_us_this = 0;
      uint8_t  sr04_valid_this = 0;
      uint8_t  sr04_attempted = ((samples_collected % PHASE4B_HCSR04_EVERY) == 0);

      if (sr04_attempted)
      {
          HCSR04_Trigger();
          HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

          uint32_t hs_start = HAL_GetTick();
          while (!echo_ready && (HAL_GetTick() - hs_start) < 30) { }

          if (echo_ready) {
              sr04_us_this = echo_pulse_us;
              sr04_valid_this = 1;
              sr04_count++;
              sr04_sum_us    += (double)sr04_us_this;
              sr04_sum_sq_us += (double)sr04_us_this * (double)sr04_us_this;
              if (sr04_us_this < sr04_min_us) sr04_min_us = sr04_us_this;
              if (sr04_us_this > sr04_max_us) sr04_max_us = sr04_us_this;
          } else {
              sr04_err_count++;
          }
          HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
      }

      /* Per-sample print */
      if (sr04_valid_this) {
          printf("%3lu,%4u,%u,%.2f,%lu\r\n",
                 (unsigned long)samples_collected,
                 m.RangeMilliMeter, m.RangeStatus,
                 signal_mcps,
                 (unsigned long)sr04_us_this);
      } else if (sr04_attempted) {
          printf("%3lu,%4u,%u,%.2f,SR04_TIMEOUT\r\n",
                 (unsigned long)samples_collected,
                 m.RangeMilliMeter, m.RangeStatus, signal_mcps);
      } else {
          printf("%3lu,%4u,%u,%.2f,-\r\n",
                 (unsigned long)samples_collected,
                 m.RangeMilliMeter, m.RangeStatus, signal_mcps);
      }

      VL53L0X_ClearInterruptMask(pVL53L0X, 0);
  }

  /* ---------------- Stop & report ---------------- */
  VL53L0X_StopMeasurement(pVL53L0X);
  Motor_Stop();
  printf("\r\n[TEST] Motor STOP, Measurement END.\r\n");

  /* Statistics */
  double vl_mean = 0.0, vl_var = 0.0, vl_std = 0.0;
  if (vl_status0_count > 1) {
      vl_mean = vl_sum / (double)vl_status0_count;
      vl_var  = (vl_sum_sq / (double)vl_status0_count) - (vl_mean * vl_mean);
      if (vl_var < 0.0) vl_var = 0.0;
      vl_std  = sqrt(vl_var);
  }

  double sr04_mean_us = 0.0, sr04_var_us = 0.0, sr04_std_us = 0.0;
  double sr04_mean_mm = 0.0, sr04_std_mm = 0.0;
  if (sr04_count > 1) {
      sr04_mean_us = sr04_sum_us / (double)sr04_count;
      sr04_var_us  = (sr04_sum_sq_us / (double)sr04_count) - (sr04_mean_us * sr04_mean_us);
      if (sr04_var_us < 0.0) sr04_var_us = 0.0;
      sr04_std_us  = sqrt(sr04_var_us);
      sr04_mean_mm = sr04_mean_us * 0.1715;  /* 343 m/s / 2 -> 0.1715 mm/us */
      sr04_std_mm  = sr04_std_us  * 0.1715;
  }

  printf("\r\n========================================\r\n");
  printf(" Phase 4-B Statistics\r\n");
  printf("========================================\r\n");
  printf(" VL53L0X (requested %lu, collected %lu)\r\n",
         (unsigned long)PHASE4B_N_SAMPLES, (unsigned long)samples_collected);
  printf("   Status0 count:   %lu/%lu\r\n",
         (unsigned long)vl_status0_count, (unsigned long)samples_collected);
  printf("   Mean dist:       %7.2f mm\r\n", vl_mean);
  printf("   Std (sigma):     %7.2f mm\r\n", vl_std);
  printf("   Min:             %5u mm\r\n", vl_min);
  printf("   Max:             %5u mm\r\n", vl_max);
  printf("   I2C errors:      %lu\r\n", (unsigned long)vl_i2c_err_count);
  printf("   Poll timeouts:   %lu\r\n", (unsigned long)poll_timeout_count);
  printf("\r\n");
  printf(" HC-SR04 (every %d samples)\r\n", PHASE4B_HCSR04_EVERY);
  printf("   Valid count:     %lu\r\n", (unsigned long)sr04_count);
  printf("   Echo timeouts:   %lu\r\n", (unsigned long)sr04_err_count);
  printf("   Mean pulse:      %7.2f us  (%7.2f mm)\r\n", sr04_mean_us, sr04_mean_mm);
  printf("   Std pulse:       %7.2f us  (%7.2f mm)\r\n", sr04_std_us, sr04_std_mm);
  if (sr04_count > 0) {
      printf("   Min pulse:       %lu us\r\n", (unsigned long)sr04_min_us);
      printf("   Max pulse:       %lu us\r\n", (unsigned long)sr04_max_us);
  }
  printf("========================================\r\n");
  printf("\r\n Baseline (Phase 1, motor OFF): sigma=1.53 mm, I2C_err=0\r\n");
  printf(" Delta sigma:  %+.2f mm\r\n", vl_std - 1.53);
  printf("========================================\r\n");
  printf("\r\n[DONE] Press RESET to run again.\r\n");

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(500);
      /* USER CODE END WHILE */
      /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* ============================================================
 * SystemClock_Config & MX_*_Init  (unchanged from previous build)
 * ============================================================ */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

  /* B1 button: input, no pull (board provides external pull-up) */
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

void HCSR04_Trigger(void)
{
    echo_capture_state = 0;
    echo_ready = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    DWT_Delay_us(10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
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

/* printf redirect to HC-06 (USART6).
 * Phase 4-B onwards: all telemetry over Bluetooth, no USB cable required. */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* TB6612FNG: both motors forward, 50% PWM */
static void Motor_Start_50pct(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_SET);    /* AIN1 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);  /* AIN2 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);    /* BIN1 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);  /* BIN2 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);    /* STBY HIGH */

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MOTOR_PWM_DUTY);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MOTOR_PWM_DUTY);
}

static void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
    /* STBY stays HIGH - no need to re-init PWM if user resets */
}

/* Block until B1 (PC13) is pressed.
 * NUCLEO B1: external pull-up, press -> GPIO_PIN_RESET. */
static void Wait_For_B1_Press(void)
{
    /* Wait for stable HIGH (in case held down at boot) */
    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(10);
    }
    /* Wait for press */
    while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {
        HAL_Delay(10);
    }
    HAL_Delay(50);  /* debounce */
    printf("[B1 pressed]\r\n");
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
