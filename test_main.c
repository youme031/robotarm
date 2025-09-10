/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : BLDC DIR+VSP + SPI1 Encoder + UART Telemetry + Pure PID
 * @note           : SPI1 8-bit + CPHA=2EDGE, PB8=TIM4_CH3 PWM(VSP), PB9=DIR
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* none */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ========= Encoder & kinematics ========= */
#define ENC_RES_COUNTS           16384U
#define GEAR_RATIO               8.0f
#define ENCODER_ON_MOTOR         1
#define GEAR_APPLY               (ENCODER_ON_MOTOR ? GEAR_RATIO : 1.0f)

/* 엔코더 각도 방향: 측정각이 CW로 증가하면 +1, 반대면 -1 */
#define ENC_DIR_SIGN             (+1)

/* ========= Control timing ========= */
#define CTRL_DT_MS               10u
#define CTRL_DT_S                (0.01f)

/* ========= PWM ========= */
#define DUTY_MAX_PERCENT         30.0f
#define DUTY_RAMP_PCT_PER_STEP   0.2f    // 10ms마다 증감 한계(부드러운 램프)

/* ========= Pure PID (position) ========= */
#define PID_KP                   1.20f
#define PID_KI                   0.010f
#define PID_KD                   0.10f    // D on measurement(=속도)
#define I_MAX                    20.0f

/* ========= Practical tweaks ========= */
#define DEAD_BAND_DEG            1.0f     // 이내면 정지 허용
#define VEL_LPF_ALPHA            0.20f    // 속도 1차 LPF(10ms 기준 8~10Hz 정도)
#define MAX_CMD_DELTA_DEG        360.0f   // 한 번의 명령으로 허용할 최대 이동(±360°)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline uint32_t PWM_GetARR(void) {
  extern TIM_HandleTypeDef htim4;
  return __HAL_TIM_GET_AUTORELOAD(&htim4);
}
static inline float wrap_deg_360(float x) {
  float y = fmodf(x, 360.0f);
  if (y < 0.0f) y += 360.0f;
  return y;
}
static inline float wrap_deg_pm180(float x) {
  float y = fmodf(x + 180.0f, 360.0f);
  if (y < 0.0f) y += 360.0f;
  return y - 180.0f;
}
static inline float angle_err_shortest(float tgt_deg, float meas_deg) {
  return wrap_deg_pm180(tgt_deg - meas_deg); // -180..+180
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static int32_t input_unwrapped_counts = 0;
static uint16_t prev_raw = 0;
static uint8_t  first_raw = 1;

static float zero_offset_deg   = 0.0f;
static float meas_deg_unwrap   = 0.0f;     // 내부 연속각
static float target_deg_unwrap = 0.0f;     // 명령(연속각)

static float I          = 0.0f;            // 단일 적분
static float last_u_01  = 0.0f;            // 램프를 위한 지난 듀티

static char    tx_buf[96];
static uint8_t rx_ch;
static char    rx_line[48];
static uint8_t rx_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */
static void ctrl_step_10ms(void);
static void process_uart_line(const char *s);
static uint16_t read_encoder_14bit(void);
static void BLDC_SetDirection(uint8_t cw);
static void BLDC_SetDutyPercent(float percent);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void BLDC_SetDutyPercent(float percent) {
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;
  uint32_t arr = PWM_GetARR();
  uint32_t ccr = (uint32_t)((percent / 100.0f) * (arr + 1) + 0.5f);
  if (ccr > arr) ccr = arr;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ccr);
}
static void BLDC_SetDirection(uint8_t cw) {
  HAL_GPIO_WritePin(GPIOB, L1_DIR_Pin, cw ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static uint16_t read_encoder_14bit(void) {
  uint8_t tx[2] = {0,0}, rx[2] = {0,0};
  HAL_GPIO_WritePin(L1_ENCODER_CS_GPIO_Port, L1_ENCODER_CS_Pin, GPIO_PIN_RESET);
  for (volatile int i=0;i<80;i++); // 짧은 지연
  HAL_SPI_TransmitReceive(&hspi1, &tx[0], &rx[0], 1, 50);
  HAL_SPI_TransmitReceive(&hspi1, &tx[1], &rx[1], 1, 50);
  HAL_GPIO_WritePin(L1_ENCODER_CS_GPIO_Port, L1_ENCODER_CS_Pin, GPIO_PIN_SET);
  uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
  return (raw & 0x3FFF);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart3) {
    char c = (char)rx_ch;
    if (c == '\n' || c == '\r') {
      rx_line[rx_len] = 0;
      if (rx_len > 0) process_uart_line(rx_line);
      rx_len = 0;
    } else {
      if (rx_len < sizeof(rx_line)-1) rx_line[rx_len++] = c;
    }
    HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
  }
}

static void process_uart_line(const char *s) {
  if (s[0]=='T' || s[0]=='t') {
    const char *p = strchr(s, ',');
    if (p != NULL && *(p+1) != '\0') {
      float v = strtof(p+1, NULL);            // 명령(0~360로 가정)
      float new_cmd_deg = wrap_deg_360(v);
      float current_deg_360 = wrap_deg_360(meas_deg_unwrap);
      float shortest_move   = angle_err_shortest(new_cmd_deg, current_deg_360);
      float new_target_unwrap = meas_deg_unwrap + shortest_move;

      // 케이블 보호용: 한 번 명령의 최대 이동 제한
      float min_allow = meas_deg_unwrap - MAX_CMD_DELTA_DEG;
      float max_allow = meas_deg_unwrap + MAX_CMD_DELTA_DEG;
      if (new_target_unwrap < min_allow) new_target_unwrap = min_allow;
      if (new_target_unwrap > max_allow) new_target_unwrap = max_allow;

      target_deg_unwrap = new_target_unwrap;

      int n = snprintf(tx_buf, sizeof(tx_buf), "ACK,TARGET=%.3f\r\n", target_deg_unwrap);
      HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, n, 10);
    }
  }
}

/* ---------- Pure PID loop (10ms) ---------- */
static void ctrl_step_10ms(void)
{
  /* 1) 엔코더 언랩 */
  uint16_t raw = read_encoder_14bit();
  if (first_raw) {
    input_unwrapped_counts = raw;
    first_raw = 0;
  } else {
    int32_t d = (int32_t)raw - (int32_t)prev_raw;
    if (d >  (int32_t)(ENC_RES_COUNTS/2)) d -= ENC_RES_COUNTS;
    if (d < -(int32_t)(ENC_RES_COUNTS/2)) d += ENC_RES_COUNTS;
    input_unwrapped_counts += d;
  }
  prev_raw = raw;

  /* 2) 기구각(연속) */
  float input_total_deg_unwrap = (float)input_unwrapped_counts * (360.0f / (float)ENC_RES_COUNTS);
  float out_deg_unbounded      = (ENC_DIR_SIGN * input_total_deg_unwrap) / GEAR_APPLY;
  meas_deg_unwrap              = out_deg_unbounded - zero_offset_deg;

  /* 3) 시각화용 */
  float meas_deg_360   = wrap_deg_360(meas_deg_unwrap);
  float target_deg_360 = wrap_deg_360(target_deg_unwrap);

  /* 4) 속도 (D on measurement) + LPF */
  static float meas_prev_360 = 0.0f;
  float dtheta  = angle_err_shortest(meas_deg_360, meas_prev_360);
  float vel     = dtheta / CTRL_DT_S;     // +CW, -CCW
  meas_prev_360 = meas_deg_360;

  static float vel_filt = 0.0f;
  vel_filt += VEL_LPF_ALPHA * (vel - vel_filt);

  /* 5) 오차(짧은 경로) */
  float err  = angle_err_shortest(target_deg_360, meas_deg_360);
  float abse = fabsf(err);

  /* 6) 적분 (근처에서는 살짝 누출) */
  if (abse > DEAD_BAND_DEG) {
    I += PID_KI * err * CTRL_DT_S;
    if (I >  I_MAX) I =  I_MAX;
    if (I < -I_MAX) I = -I_MAX;
  } else {
    I *= 0.98f;
  }

  /* 7) PID 합성 */
  float u = PID_KP * err + I - PID_KD * vel_filt;

  /* 8) 방향/크기 + 램프 + 데드밴드 */
  float mag = fabsf(u);
  if (mag > DUTY_MAX_PERCENT) mag = DUTY_MAX_PERCENT;

  if (abse <= DEAD_BAND_DEG && fabsf(vel_filt) < 1.0f) {
    BLDC_SetDutyPercent(0.0f);
    last_u_01 = 0.0f;
  } else {
    float delta = mag - last_u_01;
    float step  = DUTY_RAMP_PCT_PER_STEP;
    if (delta >  step) mag = last_u_01 + step;
    if (delta < -step) mag = last_u_01 - step;

    BLDC_SetDirection((u >= 0.0f) ? 1 : 0);
    BLDC_SetDutyPercent(mag);
    last_u_01 = mag;
  }

  /* 9) 텔레메트리 (state=0 고정) */
  int n = snprintf(tx_buf, sizeof(tx_buf), "%.3f,%.3f,%d,%.3f\r\n",
                   (double)meas_deg_360, (double)err, 0, (double)target_deg_unwrap);
  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buf, n, 5);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  BLDC_SetDirection(1);
  BLDC_SetDutyPercent(0.0f);

  uint16_t initial_raw = read_encoder_14bit();
  input_unwrapped_counts = initial_raw;
  prev_raw   = initial_raw;
  first_raw  = 0;

  float input_total_deg_unwrap = (float)input_unwrapped_counts * (360.0f / (float)ENC_RES_COUNTS);
  float out_deg_unbounded      = (ENC_DIR_SIGN * input_total_deg_unwrap) / GEAR_APPLY;
  zero_offset_deg              = out_deg_unbounded;

  meas_deg_unwrap   = 0.0f;
  target_deg_unwrap = 0.0f;

  HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
  uint32_t t0 = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    if ((now - t0) >= CTRL_DT_MS) {
      t0 += CTRL_DT_MS;     // 10ms 주기 유지
      ctrl_step_10ms();     // 제어 루프
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/* ========= Peripherals (auto-generated region 그대로) ========= */

static void MX_ETH_Init(void)
{
  static uint8_t MACAddr[6];
  heth.Instance = ETH;
  MACAddr[0] = 0x00; MACAddr[1] = 0x80; MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00; MACAddr[4] = 0x00; MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;
  if (HAL_ETH_Init(&heth) != HAL_OK) { Error_Handler(); }
  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes   = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl   = ETH_CRC_PAD_INSERT;
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7;
  hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI3_Init(void)
{
  hspi3.Instance               = SPI3;
  hspi3.Init.Mode              = SPI_MODE_MASTER;
  hspi3.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize          = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi3.Init.NSS               = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial     = 7;
  hspi3.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig     = {0};
  TIM_OC_InitTypeDef sConfigOC              = {0};

  htim4.Instance               = TIM4;
  htim4.Init.Prescaler         = 107;
  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim4.Init.Period            = 49;
  htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim4);
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance          = USART3;
  huart3.Init.BaudRate     = 115200;
  huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
}

static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1  = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_GPIO_WritePin(L1_ENCODER_CS_GPIO_Port, L1_ENCODER_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|L2_ENCODER_CS_Pin|LD2_Pin|L1_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L2_DIR_GPIO_Port, L2_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = USER_Btn_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = L1_ENCODER_CS_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L1_ENCODER_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = LD1_Pin|LD3_Pin|L2_ENCODER_CS_Pin|LD2_Pin|L1_DIR_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = L2_DIR_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L2_DIR_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed= GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
/* USER CODE END 4 */

/* Error Handler (단일 정의) */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
