/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* VL53L0X API */
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEST_1_3_SAMPLES        100
#define TEST_1_4_DURATION_MS    5000
#define TEST_1_4_PERIOD_MS      20      /* 50Hz target */
#define TEST_1_5_DURATION_MS    5000    /* per scenario */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

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

/* HC-SR04 state (Phase 3, currently unused in Phase 1) */
volatile uint8_t  echo_capture_state = 0;
volatile uint32_t echo_rising_tick   = 0;
volatile uint32_t echo_falling_tick  = 0;
volatile uint32_t echo_pulse_us      = 0;
volatile uint8_t  echo_ready         = 0;

float distance_mm = 0.0f;
uint32_t loop_max_us = 0;

/* VL53L0X device handle */
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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

  HAL_Delay(2000);  /* HC-06 boot + PuTTY open */

  /* ============================================================
   * Phase 5: HC-06 Bluetooth dummy data test
   *   Send "Hello from STM32 #N" every 1 second via USART6.
   *   PuTTY (115200 baud, BT Outgoing COM) should receive each line.
   * ============================================================ */
  printf("\r\n[Phase 5] HC-06 dummy data test starting...\r\n");
  printf("Open PuTTY on Bluetooth Outgoing COM @ 115200 baud.\r\n\r\n");

#if 0  /* === Phase 1 (VL53L0X) DISABLED for Phase 5 baud change === */

  /* ============================================================
   * Phase 1 Test 1-1: VL53L0X I2C Address Scan
   * ============================================================ */
  printf("\r\n========================================\r\n");
  printf(" Phase 1 Test 1-1: I2C Address Scan\r\n");
  printf(" Target: 0x29 (7-bit) = 0x52 (8-bit)\r\n");
  printf("========================================\r\n");

  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, (0x29 << 1), 3, 100);
  if (status == HAL_OK) {
      printf("[PASS] 0x29 responded (HAL_OK)\r\n");
  } else {
      printf("[FAIL] 0x29 NOT responding (status=%d)\r\n", status);
      Error_Handler();
  }

  /* ============================================================
   * Phase 1 Test 1-2: VL53L0X Init + Single Ranging
   * ============================================================ */
  printf("\r\n========================================\r\n");
  printf(" Phase 1 Test 1-2: Single Measurement\r\n");
  printf("========================================\r\n");

  pVL53L0X->I2cHandle  = &hi2c1;
  pVL53L0X->I2cDevAddr = 0x52;

  VL53L0X_Error vl_status = VL53L0X_ERROR_NONE;

  /* === Force device reset to ensure clean state ===
   * 이전 세션의 잔류 상태를 강제 초기화. 펌웨어 reset만으로는
   * 칩 내부 상태가 완전히 reset되지 않아 RefCalibration 실패하는 경우 방지. */
  printf("[INIT] VL53L0X_ResetDevice...         ");
  vl_status = VL53L0X_ResetDevice(pVL53L0X);
  HAL_Delay(50);  /* reset 후 칩 안정화 대기 */
  if (vl_status != VL53L0X_ERROR_NONE) {
      printf("WARN (err=%d, continuing)\r\n", vl_status);
  } else {
      printf("OK\r\n");
  }

  /* === DIAGNOSTIC: Read VL53L0X model ID and revision === */
  printf("[DIAG] Reading device identification...\r\n");
  uint8_t  model_id = 0;
  uint8_t  revision_id = 0;
  uint16_t device_id = 0;

  vl_status = VL53L0X_RdByte(pVL53L0X, 0xC0, &model_id);
  printf("       Model ID  (reg 0xC0): 0x%02X (expect 0xEE)  %s\r\n",
         model_id,
         (vl_status == VL53L0X_ERROR_NONE) ? "[OK]" : "[I2C ERR]");

  vl_status = VL53L0X_RdByte(pVL53L0X, 0xC2, &revision_id);
  printf("       Revision  (reg 0xC2): 0x%02X (expect 0x10)  %s\r\n",
         revision_id,
         (vl_status == VL53L0X_ERROR_NONE) ? "[OK]" : "[I2C ERR]");

  vl_status = VL53L0X_RdWord(pVL53L0X, 0xC0, &device_id);
  printf("       Device ID (reg 0xC0): 0x%04X (expect 0xEEAA)\r\n", device_id);
  printf("\r\n");

  printf("[INIT] DataInit...                    ");
  vl_status = VL53L0X_DataInit(pVL53L0X);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("[INIT] StaticInit...                  ");
  vl_status = VL53L0X_StaticInit(pVL53L0X);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  /* === DIAGNOSTIC: PerformRefCalibration with retry === */
  uint8_t VhvSettings = 0, PhaseCal = 0;
  int refcal_attempts = 0;
  const int REFCAL_MAX_ATTEMPTS = 3;

  for (refcal_attempts = 1; refcal_attempts <= REFCAL_MAX_ATTEMPTS; refcal_attempts++) {
      printf("[INIT] PerformRefCalibration (try %d/%d) ... ",
             refcal_attempts, REFCAL_MAX_ATTEMPTS);
      vl_status = VL53L0X_PerformRefCalibration(pVL53L0X, &VhvSettings, &PhaseCal);
      if (vl_status == VL53L0X_ERROR_NONE) {
          printf("OK (Vhv=%u, Phase=%u)\r\n", VhvSettings, PhaseCal);
          break;
      } else {
          printf("FAIL (err=%d)\r\n", vl_status);
          if (refcal_attempts < REFCAL_MAX_ATTEMPTS) {
              printf("       Retrying in 500ms...\r\n");
              HAL_Delay(500);
          }
      }
  }

  if (vl_status != VL53L0X_ERROR_NONE) {
      printf("\r\n[ABORT] RefCalibration failed after %d attempts.\r\n",
             REFCAL_MAX_ATTEMPTS);
      printf("        Diagnostic info:\r\n");
      printf("          - Model ID read: 0x%02X (expected 0xEE)\r\n", model_id);
      printf("          - DataInit/StaticInit passed but RefCal failed\r\n");
      printf("          - Likely VCSEL driver or power issue\r\n");
      Error_Handler();
  }

  printf("[INIT] PerformRefSpadManagement...    ");
  uint32_t refSpadCount = 0;
  uint8_t  isApertureSpads = 0;
  vl_status = VL53L0X_PerformRefSpadManagement(pVL53L0X, &refSpadCount, &isApertureSpads);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK (Spads=%lu, Aperture=%u)\r\n", (unsigned long)refSpadCount, isApertureSpads);

  printf("[INIT] Initialization complete.\r\n\r\n");

  VL53L0X_RangingMeasurementData_t measurement;
  for (int i = 1; i <= 5; i++) {
      vl_status = VL53L0X_PerformSingleRangingMeasurement(pVL53L0X, &measurement);
      if (vl_status == VL53L0X_ERROR_NONE) {
          printf("Measurement #%d: %u mm  (status: %u)\r\n",
                 i, measurement.RangeMilliMeter, measurement.RangeStatus);
      } else {
          printf("Measurement #%d: FAIL (err=%d)\r\n", i, vl_status);
      }
      HAL_Delay(100);
  }
  printf("\r\n[PASS] Test 1-2 complete.\r\n");

  /* ============================================================
   * Phase 1 Test 1-3: Static Repeatability (100 samples)
   * ============================================================ */
  HAL_Delay(2000);

  printf("\r\n========================================\r\n");
  printf(" Phase 1 Test 1-3: Static Repeatability\r\n");
  printf(" Target: 100 mm (white surface)\r\n");
  printf(" Samples: %d\r\n", TEST_1_3_SAMPLES);
  printf("========================================\r\n\r\n");

  uint32_t valid_count = 0;
  uint32_t status_zero_count = 0;
  double sum = 0.0, sum_sq = 0.0;
  uint16_t min_val = 0xFFFF, max_val = 0;

  for (int i = 1; i <= TEST_1_3_SAMPLES; i++) {
      vl_status = VL53L0X_PerformSingleRangingMeasurement(pVL53L0X, &measurement);
      if (vl_status == VL53L0X_ERROR_NONE) {
          uint16_t d = measurement.RangeMilliMeter;
          printf("%3d: %3u mm\r\n", i, d);
          if (measurement.RangeStatus == 0) {
              status_zero_count++;
              sum    += (double)d;
              sum_sq += (double)d * (double)d;
              if (d < min_val) min_val = d;
              if (d > max_val) max_val = d;
          }
          valid_count++;
      } else {
          printf("%3d: FAIL (err=%d)\r\n", i, vl_status);
      }
  }

  double mean_13 = 0.0, var_13 = 0.0, std_13 = 0.0;
  if (status_zero_count > 1) {
      mean_13 = sum / (double)status_zero_count;
      var_13  = (sum_sq / (double)status_zero_count) - (mean_13 * mean_13);
      if (var_13 < 0.0) var_13 = 0.0;
      std_13  = sqrt(var_13);
  }

  printf("\r\n========================================\r\n");
  printf(" Statistics (n=%d, valid=%lu, status0=%lu)\r\n",
         TEST_1_3_SAMPLES, (unsigned long)valid_count, (unsigned long)status_zero_count);
  printf(" Mean:   %7.2f mm\r\n", mean_13);
  printf(" Std:    %7.2f mm\r\n", std_13);
  printf(" Var:    %7.2f mm^2\r\n", var_13);
  printf(" Min:    %5u mm\r\n", min_val);
  printf(" Max:    %5u mm\r\n", max_val);
  printf("========================================\r\n");

  /* Test 1-3은 표준편차 기준으로 PASS 판정 (오프셋은 측정 기준점 차이) */
  if (std_13 <= 5.0 && status_zero_count >= (TEST_1_3_SAMPLES * 9 / 10)) {
      printf("\r\n[PASS] Test 1-3 complete.\r\n");
      printf("  Std %.2f mm <= 5 mm (target)\r\n", std_13);
  } else {
      printf("\r\n[FAIL] Test 1-3: std %.2f mm > 5 mm or status0 count too low\r\n", std_13);
  }

  /* ============================================================
   * Phase 1 Test 1-4: Continuous Mode + Polling Rate Verification
   * ============================================================ */
  HAL_Delay(2000);

  printf("\r\n========================================\r\n");
  printf(" Phase 1 Test 1-4: Continuous Mode\r\n");
  printf(" Target rate: 50 Hz (period %d ms)\r\n", TEST_1_4_PERIOD_MS);
  printf(" Duration: %d ms\r\n", TEST_1_4_DURATION_MS);
  printf("========================================\r\n");

  printf("[INIT] SetMeasurementTimingBudget 20ms ... ");
  vl_status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pVL53L0X, 20000);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK (high-speed mode)\r\n");

  printf("[INIT] SetDeviceMode CONTINUOUS_RANGING ... ");
  vl_status = VL53L0X_SetDeviceMode(pVL53L0X, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("[INIT] SetInterMeasurementPeriod %dms   ... ", TEST_1_4_PERIOD_MS);
  vl_status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(pVL53L0X, TEST_1_4_PERIOD_MS);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n");

  printf("[INIT] StartMeasurement                 ... ");
  vl_status = VL53L0X_StartMeasurement(pVL53L0X);
  if (vl_status != VL53L0X_ERROR_NONE) { printf("FAIL (err=%d)\r\n", vl_status); Error_Handler(); }
  printf("OK\r\n\r\n");

  printf("Polling for new data...\r\n\r\n");

  uint32_t test_start_ms = HAL_GetTick();
  uint32_t test_end_ms   = test_start_ms + TEST_1_4_DURATION_MS;
  uint32_t sample_count  = 0;
  uint32_t last_data_ms  = test_start_ms;
  uint32_t interval_sum  = 0;
  uint32_t interval_min  = 0xFFFFFFFF;
  uint32_t interval_max  = 0;

  while (HAL_GetTick() < test_end_ms)
  {
      uint8_t data_ready = 0;
      vl_status = VL53L0X_GetMeasurementDataReady(pVL53L0X, &data_ready);

      if (vl_status == VL53L0X_ERROR_NONE && data_ready)
      {
          uint32_t now_ms = HAL_GetTick();
          uint32_t delta_from_start = now_ms - test_start_ms;
          uint32_t interval = now_ms - last_data_ms;

          /* 첫 샘플은 인터벌 통계에서 제외 (시작점 영향) */
          if (sample_count >= 1) {
              interval_sum += interval;
              if (interval < interval_min) interval_min = interval;
              if (interval > interval_max) interval_max = interval;
          }
          last_data_ms = now_ms;

          /* 측정값 읽기 */
          VL53L0X_RangingMeasurementData_t cont_meas;
          vl_status = VL53L0X_GetRangingMeasurementData(pVL53L0X, &cont_meas);
          if (vl_status == VL53L0X_ERROR_NONE) {
              sample_count++;

              /* 출력은 일부 샘플만 (가독성) */
              if (sample_count == 1 || sample_count == 10 ||
                  sample_count % 50 == 0) {
                  printf("Sample %3lu: %3u mm  (delta from start: %lu ms)\r\n",
                         (unsigned long)sample_count,
                         cont_meas.RangeMilliMeter,
                         (unsigned long)delta_from_start);
              }
          }

          /* 인터럽트 마스크 클리어 (다음 측정 트리거) */
          VL53L0X_ClearInterruptMask(pVL53L0X, 0);
      }
  }

  /* Continuous 모드 종료 */
  VL53L0X_StopMeasurement(pVL53L0X);

  /* 인터벌 통계 계산 */
  double mean_interval = 0.0;
  double effective_rate = 0.0;
  if (sample_count >= 2) {
      mean_interval = (double)interval_sum / (double)(sample_count - 1);
      if (mean_interval > 0.0) {
          effective_rate = 1000.0 / mean_interval;
      }
  }

  printf("\r\n========================================\r\n");
  printf(" Statistics over %d ms\r\n", TEST_1_4_DURATION_MS);
  printf(" Total samples:    %lu\r\n", (unsigned long)sample_count);
  printf(" Effective rate:   %.2f Hz\r\n", effective_rate);
  printf(" Mean interval:    %.2f ms\r\n", mean_interval);
  if (sample_count >= 2) {
      printf(" Min interval:     %lu ms\r\n", (unsigned long)interval_min);
      printf(" Max interval:     %lu ms\r\n", (unsigned long)interval_max);
  }
  printf("========================================\r\n");

  /* Test 1-4 Pass 기준: 45-55 Hz */
  if (effective_rate >= 45.0 && effective_rate <= 55.0) {
      printf("\r\n[PASS] Test 1-4 complete.\r\n");
      printf("  Effective rate %.2f Hz within [45, 55] Hz\r\n", effective_rate);
  } else if (sample_count == 0) {
      printf("\r\n[FAIL] Test 1-4: No samples received\r\n");
  } else {
      printf("\r\n[WARN] Test 1-4: rate %.2f Hz out of [45, 55] Hz\r\n", effective_rate);
  }
  printf("\r\n");

  /* ============================================================
   * Phase 1 Test 1-5: Signal Rate & Range Status
   *   3 scenarios: White / Dark / Empty
   *   Continuous mode, 5 seconds each
   * ============================================================ */
  /* Test 1-4가 Continuous 모드를 켰으니, 그대로 사용
   * (StopMeasurement는 Test 1-4 끝에서 이미 호출됨) */

  printf("\r\n========================================\r\n");
  printf(" Phase 1 Test 1-5: Signal Rate & Status\r\n");
  printf(" 3 scenarios x %d ms each\r\n", TEST_1_5_DURATION_MS);
  printf("========================================\r\n");

  /* Continuous 모드 다시 시작 */
  vl_status = VL53L0X_StartMeasurement(pVL53L0X);
  if (vl_status != VL53L0X_ERROR_NONE) {
      printf("[ABORT] StartMeasurement failed (err=%d)\r\n", vl_status);
      Error_Handler();
  }

  /* 각 시나리오용 통계 변수 */
  typedef struct {
      const char *name;
      uint32_t sample_count;
      uint32_t status0_count;
      double   sum_dist;
      double   sum_signal;
      double   sum_ambient;
  } scenario_stats_t;

  scenario_stats_t stats[3] = {
      {"White surface @ 100mm", 0, 0, 0.0, 0.0, 0.0},
      {"Dark surface @ 100mm",  0, 0, 0.0, 0.0, 0.0},
      {"Empty field (no surface)", 0, 0, 0.0, 0.0, 0.0}
  };

  const char *scenario_prompts[3] = {
      ">>> Place WHITE surface @ 100mm. Starting in 3 seconds...",
      ">>> Replace with DARK surface @ 100mm. Starting in 5 seconds...",
      ">>> REMOVE surface (clear field). Starting in 5 seconds..."
  };

  uint32_t scenario_wait_ms[3] = {3000, 5000, 5000};

  for (int s = 0; s < 3; s++) {
      printf("\r\n%s\r\n", scenario_prompts[s]);
      HAL_Delay(scenario_wait_ms[s]);

      printf("\r\nScenario %c: %s\r\n", 'A' + s, stats[s].name);
      printf("--------------------------------------------------\r\n");

      uint32_t scen_start = HAL_GetTick();
      uint32_t scen_end   = scen_start + TEST_1_5_DURATION_MS;

      while (HAL_GetTick() < scen_end) {
          uint8_t data_ready = 0;
          vl_status = VL53L0X_GetMeasurementDataReady(pVL53L0X, &data_ready);

          if (vl_status == VL53L0X_ERROR_NONE && data_ready) {
              VL53L0X_RangingMeasurementData_t m;
              vl_status = VL53L0X_GetRangingMeasurementData(pVL53L0X, &m);
              if (vl_status == VL53L0X_ERROR_NONE) {
                  stats[s].sample_count++;

                  /* SignalRate, AmbientRate는 fixed-point 9.7 (단위: MCps) */
                  float signal_mcps  = (float)m.SignalRateRtnMegaCps / 65536.0f;
                  float ambient_mcps = (float)m.AmbientRateRtnMegaCps / 65536.0f;

                  if (m.RangeStatus == 0) {
                      stats[s].status0_count++;
                      stats[s].sum_dist   += (double)m.RangeMilliMeter;
                  }
                  stats[s].sum_signal  += (double)signal_mcps;
                  stats[s].sum_ambient += (double)ambient_mcps;

                  /* 출력은 일부만 (가독성) */
                  if (stats[s].sample_count == 1 ||
                      stats[s].sample_count == 50 ||
                      stats[s].sample_count == 100 ||
                      stats[s].sample_count % 100 == 0) {
                      printf("Sample %3lu: dist=%5u mm, status=%u, signal=%6.2f MCps, ambient=%5.2f MCps\r\n",
                             (unsigned long)stats[s].sample_count,
                             m.RangeMilliMeter,
                             m.RangeStatus,
                             signal_mcps,
                             ambient_mcps);
                  }
              }
              VL53L0X_ClearInterruptMask(pVL53L0X, 0);
          }
      }

      /* 시나리오 통계 */
      double mean_dist = 0.0, mean_signal = 0.0, mean_ambient = 0.0;
      if (stats[s].status0_count > 0) {
          mean_dist = stats[s].sum_dist / (double)stats[s].status0_count;
      }
      if (stats[s].sample_count > 0) {
          mean_signal  = stats[s].sum_signal  / (double)stats[s].sample_count;
          mean_ambient = stats[s].sum_ambient / (double)stats[s].sample_count;
      }

      printf("\r\n[Stats %c] %s\r\n", 'A' + s, stats[s].name);
      printf("  Samples:       %lu\r\n", (unsigned long)stats[s].sample_count);
      printf("  Status0 count: %lu/%lu\r\n",
             (unsigned long)stats[s].status0_count,
             (unsigned long)stats[s].sample_count);
      printf("  Mean dist:     %.2f mm (status=0 only)\r\n", mean_dist);
      printf("  Mean signal:   %.2f MCps\r\n", mean_signal);
      printf("  Mean ambient:  %.2f MCps\r\n", mean_ambient);
  }

  /* Continuous 모드 종료 */
  VL53L0X_StopMeasurement(pVL53L0X);

  /* 종합 결과 표 */
  double sig_a = (stats[0].sample_count > 0) ?
                 (stats[0].sum_signal / (double)stats[0].sample_count) : 0.0;
  double sig_b = (stats[1].sample_count > 0) ?
                 (stats[1].sum_signal / (double)stats[1].sample_count) : 0.0;
  double sig_c = (stats[2].sample_count > 0) ?
                 (stats[2].sum_signal / (double)stats[2].sample_count) : 0.0;

  printf("\r\n========================================\r\n");
  printf(" Test 1-5 Summary\r\n");
  printf("========================================\r\n");
  printf("| Scenario       | signal MCps | status0     |\r\n");
  printf("|----------------|-------------|-------------|\r\n");
  printf("| A: White       |   %7.2f   | %4lu/%4lu  |\r\n",
         sig_a, (unsigned long)stats[0].status0_count, (unsigned long)stats[0].sample_count);
  printf("| B: Dark        |   %7.2f   | %4lu/%4lu  |\r\n",
         sig_b, (unsigned long)stats[1].status0_count, (unsigned long)stats[1].sample_count);
  printf("| C: Empty       |   %7.2f   | %4lu/%4lu  |\r\n",
         sig_c, (unsigned long)stats[2].status0_count, (unsigned long)stats[2].sample_count);
  printf("\r\n");
  printf(" Expected: signal(White) > signal(Dark) > signal(Empty)\r\n");

  /* Pass/Fail 판정: signal rate가 표면 반사율에 따라 단조 감소 */
  if (sig_a > sig_b && sig_b > sig_c) {
      printf("\r\n[PASS] Test 1-5 complete.\r\n");
      printf("  Signal rate ordering correct: A > B > C\r\n");
  } else {
      printf("\r\n[WARN] Test 1-5: Signal ordering not strictly A > B > C\r\n");
      printf("       (May still be acceptable if surfaces had similar reflectance)\r\n");
  }
  printf("\r\n");

  /* ============================================================
   * Phase 1 Test 1-6: I2C Error Recovery
   *   E1: Wrong I2C address
   *   E2: Forced Stop/Restart in Continuous mode
   *   E3: Invalid register read
   * ============================================================ */
  printf("\r\n========================================\r\n");
  printf(" Phase 1 Test 1-6: I2C Error Recovery\r\n");
  printf("========================================\r\n");

  /* Continuous 모드 종료 후 Single 모드로 복귀 (Test 1-6은 Single 사용) */
  VL53L0X_StopMeasurement(pVL53L0X);
  HAL_Delay(50);
  VL53L0X_SetDeviceMode(pVL53L0X, VL53L0X_DEVICEMODE_SINGLE_RANGING);

  /* 결과 추적용 */
  int e1_pass = 0, e2_pass = 0, e3_pass = 0;

  /* ---- Pre-error baseline ---- */
  printf("\r\n[Phase] Pre-error baseline...\r\n");
  uint16_t baseline_sum = 0;
  int baseline_ok = 0;
  for (int i = 0; i < 5; i++) {
      vl_status = VL53L0X_PerformSingleRangingMeasurement(pVL53L0X, &measurement);
      if (vl_status == VL53L0X_ERROR_NONE && measurement.RangeStatus == 0) {
          baseline_sum += measurement.RangeMilliMeter;
          baseline_ok++;
      }
      HAL_Delay(50);
  }
  printf("  %d/5 measurements OK", baseline_ok);
  if (baseline_ok > 0) {
      printf(", mean dist: %u mm\r\n", baseline_sum / baseline_ok);
  } else {
      printf("\r\n");
  }

  /* ---- E1: Wrong I2C address ---- */
  printf("\r\n[E1] Inject: wrong I2C address (0x55)\r\n");

  uint8_t dummy_data = 0xC0;
  HAL_StatusTypeDef hal_status;

  /* 일부러 0x55 (8-bit 0xAA) 주소로 통신 시도 — VL53L0X 아님, 응답 없음 */
  hal_status = HAL_I2C_Master_Transmit(&hi2c1, (0x55 << 1), &dummy_data, 1, 100);
  printf("  HAL status after wrong addr: ");
  if (hal_status == HAL_OK) {
      printf("HAL_OK (unexpected, but continuing)\r\n");
  } else if (hal_status == HAL_ERROR) {
      printf("HAL_ERROR (expected NACK)\r\n");
  } else if (hal_status == HAL_TIMEOUT) {
      printf("HAL_TIMEOUT (expected)\r\n");
  } else {
      printf("HAL_BUSY or other (status=%d)\r\n", hal_status);
  }

  /* I2C 버스가 깨진 상태일 수 있음 - 복구 시도
   * STM32 HAL은 NACK 후 자동으로 STOP 비트 보내지 않을 수 있음
   * HAL_I2C_DeInit/Init으로 강제 복구하거나, 그대로 다음 측정 시도 */
  HAL_Delay(10);

  /* 복구 검증: 정상 주소로 5회 측정 */
  int e1_recovery_ok = 0;
  uint32_t e1_sum = 0;
  for (int i = 0; i < 5; i++) {
      vl_status = VL53L0X_PerformSingleRangingMeasurement(pVL53L0X, &measurement);
      if (vl_status == VL53L0X_ERROR_NONE && measurement.RangeStatus == 0) {
          e1_sum += measurement.RangeMilliMeter;
          e1_recovery_ok++;
      }
      HAL_Delay(50);
  }
  printf("  Recovery: %d/5 measurements OK", e1_recovery_ok);
  if (e1_recovery_ok > 0) {
      printf(", mean dist: %lu mm\r\n", (unsigned long)(e1_sum / e1_recovery_ok));
  } else {
      printf("\r\n");
  }

  if (e1_recovery_ok >= 4) {
      printf("  [PASS] E1\r\n");
      e1_pass = 1;
  } else {
      printf("  [FAIL] E1: %d/5 not enough\r\n", e1_recovery_ok);
  }

  /* ---- E2: Forced Stop/Restart in Continuous mode ---- */
  printf("\r\n[E2] Inject: forced StopMeasurement during Continuous\r\n");

  /* Continuous 모드 진입 */
  vl_status = VL53L0X_SetDeviceMode(pVL53L0X, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  vl_status = VL53L0X_StartMeasurement(pVL53L0X);
  HAL_Delay(100);  /* 측정 1~2회 발생할 시간 */

  /* 강제 Stop */
  vl_status = VL53L0X_StopMeasurement(pVL53L0X);
  HAL_Delay(50);

  /* 즉시 재시작 */
  vl_status = VL53L0X_StartMeasurement(pVL53L0X);
  printf("  Continuous restart status: ");
  if (vl_status == VL53L0X_ERROR_NONE) {
      printf("VL53L0X_ERROR_NONE\r\n");
  } else {
      printf("err=%d\r\n", vl_status);
  }

  /* 5개 샘플 받기 */
  int e2_recovery_ok = 0;
  uint32_t e2_sum = 0;
  uint32_t e2_start = HAL_GetTick();
  while ((HAL_GetTick() - e2_start) < 1000 && e2_recovery_ok < 5) {
      uint8_t data_ready = 0;
      VL53L0X_GetMeasurementDataReady(pVL53L0X, &data_ready);
      if (data_ready) {
          VL53L0X_RangingMeasurementData_t m;
          if (VL53L0X_GetRangingMeasurementData(pVL53L0X, &m) == VL53L0X_ERROR_NONE) {
              if (m.RangeStatus == 0) {
                  e2_sum += m.RangeMilliMeter;
                  e2_recovery_ok++;
              }
          }
          VL53L0X_ClearInterruptMask(pVL53L0X, 0);
      }
  }

  /* Continuous 종료, Single 모드 복귀 */
  VL53L0X_StopMeasurement(pVL53L0X);
  HAL_Delay(50);
  VL53L0X_SetDeviceMode(pVL53L0X, VL53L0X_DEVICEMODE_SINGLE_RANGING);

  printf("  Recovery: %d/5 measurements OK", e2_recovery_ok);
  if (e2_recovery_ok > 0) {
      printf(", mean dist: %lu mm\r\n", (unsigned long)(e2_sum / e2_recovery_ok));
  } else {
      printf("\r\n");
  }

  if (e2_recovery_ok >= 4) {
      printf("  [PASS] E2\r\n");
      e2_pass = 1;
  } else {
      printf("  [FAIL] E2: %d/5 not enough\r\n", e2_recovery_ok);
  }

  /* ---- E3: Invalid register read ---- */
  printf("\r\n[E3] Inject: read invalid register (0xFF)\r\n");

  uint8_t garbage = 0;
  vl_status = VL53L0X_RdByte(pVL53L0X, 0xFF, &garbage);
  printf("  RdByte 0xFF status: ");
  if (vl_status == VL53L0X_ERROR_NONE) {
      printf("VL53L0X_ERROR_NONE (data=0x%02X, possibly garbage)\r\n", garbage);
  } else {
      printf("err=%d (expected non-zero)\r\n", vl_status);
  }

  /* 복구 검증 */
  int e3_recovery_ok = 0;
  uint32_t e3_sum = 0;
  for (int i = 0; i < 5; i++) {
      vl_status = VL53L0X_PerformSingleRangingMeasurement(pVL53L0X, &measurement);
      if (vl_status == VL53L0X_ERROR_NONE && measurement.RangeStatus == 0) {
          e3_sum += measurement.RangeMilliMeter;
          e3_recovery_ok++;
      }
      HAL_Delay(50);
  }
  printf("  Recovery: %d/5 measurements OK", e3_recovery_ok);
  if (e3_recovery_ok > 0) {
      printf(", mean dist: %lu mm\r\n", (unsigned long)(e3_sum / e3_recovery_ok));
  } else {
      printf("\r\n");
  }

  if (e3_recovery_ok >= 4) {
      printf("  [PASS] E3\r\n");
      e3_pass = 1;
  } else {
      printf("  [FAIL] E3: %d/5 not enough\r\n", e3_recovery_ok);
  }

  /* ---- Test 1-6 Summary ---- */
  printf("\r\n========================================\r\n");
  printf(" Test 1-6 Summary\r\n");
  printf("========================================\r\n");
  printf("| Error | Recovery |\r\n");
  printf("|-------|----------|\r\n");
  printf("| E1    |   %s   |\r\n", e1_pass ? "PASS" : "FAIL");
  printf("| E2    |   %s   |\r\n", e2_pass ? "PASS" : "FAIL");
  printf("| E3    |   %s   |\r\n", e3_pass ? "PASS" : "FAIL");

  if (e1_pass && e2_pass && e3_pass) {
      printf("\r\n[PASS] Test 1-6 complete.\r\n");
      printf("  Firmware recovers from all injected I2C errors.\r\n");
  } else {
      printf("\r\n[FAIL] Test 1-6: %d/3 recovery scenarios passed\r\n",
             e1_pass + e2_pass + e3_pass);
  }
  printf("\r\n");

  /* ============================================================
   * Phase 1 Complete
   * ============================================================ */
  printf("\r\n========================================\r\n");
  printf(" Phase 1 Verification Complete\r\n");
  printf("========================================\r\n");
  printf("\r\n");

#endif  /* === END Phase 1 disabled === */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t loop_count = 0;
  uint32_t last_blink_ms = 0;

  while (1)
    {
        /* === 50Hz CSV DMA send === */
        if (HAL_GetTick() - last_blink_ms >= 20)  /* 20ms = 50Hz */
        {
            last_blink_ms = HAL_GetTick();
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            loop_count++;

            static uint32_t drop_count = 0;

            /* Build 18-field CSV (dummy values for now) */
            static char csv_buf[256];
            int len = snprintf(csv_buf, sizeof(csv_buf),
                "%lu,%lu,"           /* 1: seq, 2: timestamp_ms */
                "0,0,"               /* 3: enc_left, 4: enc_right */
                "0.0,0.0,"           /* 5: pos_left_mm, 6: pos_right_mm */
                "0,0,"               /* 7: tof_dist_mm, 8: tof_status */
                "0.0,0.0,"           /* 9: tof_signal, 10: tof_ambient */
                "0.0,0.0,"           /* 11: kf_estimate, 12: kf_covariance */
                "0.0,0.0,"           /* 13: residual, 14: residual_var */
                "0.0,"               /* 15: residual_mean */
                "0.0,0.0,"           /* 16: kalman_gain, 17: innovation_cov */
                "0\r\n",             /* 18: scenario_id */
                (unsigned long)loop_count,
                (unsigned long)HAL_GetTick());

            /* Try DMA, count if previous transmission still in progress */
            HAL_StatusTypeDef tx_st = HAL_UART_Transmit_DMA(&huart6, (uint8_t*)csv_buf, len);
            if (tx_st != HAL_OK) {
                drop_count++;
            }

            /* DEBUG: print stats every 50 samples (1 second) to ST-LINK */
            if (loop_count % 50 == 0) {
                printf("CSV sent: %lu, dropped: %lu\r\n",
                       (unsigned long)loop_count,
                       (unsigned long)drop_count);
            }
        }

      /* USER CODE END WHILE */
      /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
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
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
