/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (MOTOR STANDALONE TEST)
  * @brief          : Phase 4-A motor test via ST-LINK USB Virtual COM Port
  *
  * Hardware: NUCLEO-F446RE + JMOD-MOTOR-1 (TB6612FNG) + FIT0450 motor
  * Power: LiPo 7.4V → JMOD VIN (motor power, direct), NUCLEO 5V → JMOD +5V (logic)
  *
  * Pin mapping:
  *   TIM1_CH1 (PA8)  → JMOD PWMA   (motor speed)
  *   GPIO PC8        → JMOD AIN1   (direction 1)
  *   GPIO PC9        → JMOD AIN2   (direction 2)
  *   GPIO PC12       → JMOD STBY   (enable, HIGH = active)
  *   USART2 TX (PA2) → ST-LINK VCP (115200 baud, via NUCLEO USB)
  *
  * Serial commands (single character):
  *   i  Init       : STBY=H, AIN1=H AIN2=L (forward), PWM=0
  *   s  SoftStart  : PWM 0 → 30% over 1 second (gentle ramp)
  *   +  Speed up   : PWM +10%
  *   -  Speed down : PWM -10%
  *   r  Reverse    : toggle direction (keeps current PWM)
  *   b  Brake      : PWM=0, AIN1=H AIN2=H (short brake)
  *   x  Stop       : STBY=LOW (full stop, motor coasts)
  *   ?  Status     : print current PWM% and direction
  *
  * Safety: boots with STBY=LOW and PWM=0. Motor never moves until 'i' is sent.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define PWM_PERIOD       1000   // TIM1 ARR; duty% = (CCR / PERIOD) * 100
#define PWM_STEP         10     // % change per +/- press
#define PWM_MAX          80     // hard cap, motor can survive 100% but 80% is safer for first test
#define SOFT_START_TARGET 30    // % to reach during SoftStart
#define SOFT_START_MS    1000   // ramp duration in ms

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* Stub variables — referenced by stm32f4xx_it.c (interrupt handlers).
 * These peripherals are NOT initialized in this motor-test firmware,
 * but the original interrupt handlers expect these symbols to exist.
 * The corresponding interrupts won't fire because we don't enable them.
 */
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_usart2_tx;

static volatile uint8_t  rx_byte = 0;
static volatile uint8_t  rx_ready = 0;

static int  current_pwm_percent = 0;   // 0..100
static int  direction_forward   = 1;   // 1 = forward (AIN1=H AIN2=L), 0 = reverse
static int  motor_active        = 0;   // STBY state mirror

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);

static void uart_print(const char *s);
static void set_pwm_percent(int pct);
static void set_direction(int forward);
static void motor_init(void);
static void motor_brake(void);
static void motor_stop(void);
static void soft_start(void);
static void print_status(void);
static void handle_command(char c);

/* printf retarget -----------------------------------------------------------*/
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}

/* UART RX callback ----------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        rx_ready = 1;
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);
    }
}

/* MAIN ----------------------------------------------------------------------*/
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();

    // Force safe state at boot
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // STBY = LOW
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);  // AIN1 = LOW
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);  // AIN2 = LOW
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);        // PWM = 0%

    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte, 1);

    HAL_Delay(200);
    uart_print("\r\n=== Motor Standalone Test ===\r\n");
    uart_print("Commands: i=init, s=softstart, +/-=speed, r=reverse, b=brake, x=stop, ?=status\r\n");
    uart_print("Boot state: STBY=LOW, PWM=0 (motor will NOT move until 'i')\r\n> ");

    while (1) {
        if (rx_ready) {
            char c = (char)rx_byte;
            rx_ready = 0;
            handle_command(c);
        }
    }
}

/* Command handler -----------------------------------------------------------*/
static void handle_command(char c) {
    switch (c) {
        case 'i': case 'I':
            motor_init();
            break;
        case 's': case 'S':
            if (!motor_active) { uart_print("ERR: run 'i' first\r\n> "); return; }
            soft_start();
            break;
        case '+':
            if (!motor_active) { uart_print("ERR: run 'i' first\r\n> "); return; }
            current_pwm_percent += PWM_STEP;
            if (current_pwm_percent > PWM_MAX) current_pwm_percent = PWM_MAX;
            set_pwm_percent(current_pwm_percent);
            print_status();
            break;
        case '-':
            if (!motor_active) { uart_print("ERR: run 'i' first\r\n> "); return; }
            current_pwm_percent -= PWM_STEP;
            if (current_pwm_percent < 0) current_pwm_percent = 0;
            set_pwm_percent(current_pwm_percent);
            print_status();
            break;
        case 'r': case 'R':
            if (!motor_active) { uart_print("ERR: run 'i' first\r\n> "); return; }
            // safety: ramp down before reverse
            set_pwm_percent(0);
            HAL_Delay(200);
            direction_forward = !direction_forward;
            set_direction(direction_forward);
            HAL_Delay(50);
            set_pwm_percent(current_pwm_percent);
            print_status();
            break;
        case 'b': case 'B':
            motor_brake();
            break;
        case 'x': case 'X':
            motor_stop();
            break;
        case '?':
            print_status();
            break;
        case '\r': case '\n': case ' ':
            // ignore whitespace silently
            break;
        default: {
            char buf[32];
            snprintf(buf, sizeof(buf), "Unknown: '%c'\r\n> ", c);
            uart_print(buf);
            break;
        }
    }
}

/* Motor control primitives --------------------------------------------------*/
static void set_pwm_percent(int pct) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    uint32_t ccr = (uint32_t)((PWM_PERIOD * pct) / 100);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    current_pwm_percent = pct;
}

static void set_direction(int forward) {
    if (forward) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);    // AIN1 = H
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // AIN2 = L
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);  // AIN1 = L
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);    // AIN2 = H
    }
    direction_forward = forward;
}

static void motor_init(void) {
    set_pwm_percent(0);
    set_direction(1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);  // STBY = HIGH
    motor_active = 1;
    uart_print("Init: STBY=H, dir=FWD, PWM=0\r\n> ");
}

static void motor_brake(void) {
    set_pwm_percent(0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);  // AIN1 = H
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  // AIN2 = H (short brake)
    uart_print("Brake (short brake mode)\r\n> ");
}

static void motor_stop(void) {
    set_pwm_percent(0);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // STBY = LOW
    motor_active = 0;
    uart_print("STOP: STBY=LOW, motor disabled\r\n> ");
}

static void soft_start(void) {
    int steps = 30;
    int delay_ms = SOFT_START_MS / steps;
    int target = SOFT_START_TARGET;
    uart_print("SoftStart: ramping 0 -> 30%...\r\n");
    for (int i = 1; i <= steps; i++) {
        int pct = (target * i) / steps;
        set_pwm_percent(pct);
        HAL_Delay(delay_ms);
    }
    print_status();
}

static void print_status(void) {
    char buf[64];
    snprintf(buf, sizeof(buf), "PWM=%d%%, dir=%s, STBY=%s\r\n> ",
             current_pwm_percent,
             direction_forward ? "FWD" : "REV",
             motor_active ? "H" : "L");
    uart_print(buf);
}

static void uart_print(const char *s) {
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

/* ===========================================================================
 * Peripheral init (auto-generated style — keep in sync with .ioc)
 * =========================================================================== */

void SystemClock_Config(void) {
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
    RCC_OscInitStruct.PLL.PLLQ = 7;
    RCC_OscInitStruct.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 17;        // 180MHz / (17+1) = 10MHz
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD - 1;  // 10MHz / 1000 = 10kHz PWM
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    HAL_TIM_MspPostInit(&htim1);
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
