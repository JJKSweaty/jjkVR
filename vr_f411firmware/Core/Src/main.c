/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "icm20948.h"
#include "pose_fusion.h"
#include "tfluna_uart.h"
#include "usbd_hid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef ENABLE_LIDAR_FORWARD_FUSION
#define ENABLE_LIDAR_FORWARD_FUSION  1
#endif
#ifndef ENABLE_IMU_DEMO_POSITION
#define ENABLE_IMU_DEMO_POSITION     1
#endif

#define IMU_PERIOD_MS              5U
#define HID_PERIOD_MS             10U
#define LIDAR_PERIOD_MS           50U
#define GYRO_CALIBRATION_SAMPLES 200U
#define TFLUNA_UART_BAUD         115200U
#define TFLUNA_CONNECTION_TIMEOUT_MS 250U
#define TFLUNA_MIN_CM             20U
#define TFLUNA_MAX_CM            800U
#define TFLUNA_MIN_AMPLITUDE     100U
#define LIDAR_MIN_FORWARD_COSINE   0.70f
#define LIDAR_MAX_DISAGREEMENT_M   0.35f
#define LIDAR_BASELINE_SAMPLES       8U
/* Retain 90% of the fast IMU prediction per 20 Hz LiDAR correction. */
#define LIDAR_CORRECTION_WEIGHT    0.10f

#define HID_REPORT_SIZE           64U
#define HID_QUATERNION_OFFSET      1U
#define HID_POSITION_OFFSET       17U
#define HID_FLAGS_OFFSET          29U
#define HID_VERSION_OFFSET        30U
#define HID_STATUS_OFFSET         31U
#define HID_PROTOCOL_VERSION       2U
#define HID_POSITION_VALID         0x01U
#define HID_LIDAR_FUSED             0x02U

/* Fail loudly if CubeMX regenerates its 4-byte mouse report over this protocol. */
_Static_assert(HID_EPIN_SIZE == HID_REPORT_SIZE, "JJKVR HID endpoint must be 64 bytes");
_Static_assert(HID_MOUSE_REPORT_DESC_SIZE == 23U, "Relativty HID descriptor was overwritten");
_Static_assert(sizeof(float) == 4U, "JJKVR HID protocol requires 32-bit float");

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static Icm20948 imu;
static PoseFusion pose_fusion;
static uint8_t sensor_status;
static float lidar_baseline_m;
static uint8_t lidar_baseline_samples;
static TfLunaUartParser lidar_parser;
static volatile uint16_t lidar_distance_cm;
static volatile uint16_t lidar_amplitude;
static volatile uint32_t lidar_last_frame_ms;
static volatile bool lidar_sample_ready;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void leds_off(void);
static void show_status(GPIO_TypeDef *port, uint16_t pin);
static void show_sensor_leds(bool imu_connected, bool lidar_connected);
static void recover_i2c_bus(void);
static void init_lidar_uart(void);
static void poll_lidar_uart(void);
static bool lidar_is_connected(uint32_t now_ms);
static void send_startup_status(void);
static void send_pose_report(bool lidar_fused);
#if ENABLE_LIDAR_FORWARD_FUSION
static bool update_lidar_forward(uint16_t distance_cm, uint16_t amplitude);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void leds_off(void)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

static void show_status(GPIO_TypeDef *port, uint16_t pin)
{
  leds_off();
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

static void show_sensor_leds(bool imu_connected, bool lidar_connected)
{
  leds_off();
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,
                   imu_connected ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,
                   lidar_connected ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void recover_i2c_bus(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* A timed-out slave can leave F4 I2C BUSY set; reset it before either sensor retries. */
  SET_BIT(hi2c1.Instance->CR1, I2C_CR1_SWRST);
  CLEAR_BIT(hi2c1.Instance->CR1, I2C_CR1_SWRST);
  (void)HAL_I2C_DeInit(&hi2c1);
  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  /* Release a slave stuck mid-byte, then generate a STOP on PB6/PB7. */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
  for (uint32_t pulse = 0U;
       pulse < 9U && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET;
       pulse++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1U);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1U);
  MX_I2C1_Init();
}

static void init_lidar_uart(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* Receive-only is enough for the TF-Luna's default continuous 9-byte stream. */
  USART2->CR1 = 0U;
  USART2->CR2 = 0U;
  USART2->CR3 = 0U;
  USART2->BRR = (HAL_RCC_GetPCLK1Freq() + TFLUNA_UART_BAUD / 2U) /
                TFLUNA_UART_BAUD;
  HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  USART2->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
}

static void poll_lidar_uart(void)
{
  while ((USART2->SR & USART_SR_RXNE) != 0U)
  {
    uint32_t status = USART2->SR;
    uint8_t byte = (uint8_t)USART2->DR;

    if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
      lidar_parser.index = 0U;
      continue;
    }
    uint16_t distance_cm;
    uint16_t amplitude;
    if (TfLunaUart_Feed(&lidar_parser, byte, &distance_cm, &amplitude))
    {
      lidar_distance_cm = distance_cm;
      lidar_amplitude = amplitude;
      lidar_last_frame_ms = HAL_GetTick();
      lidar_sample_ready = true;
    }
  }
}

static bool lidar_is_connected(uint32_t now_ms)
{
  return lidar_last_frame_ms != 0U &&
         (uint32_t)(now_ms - lidar_last_frame_ms) <=
             TFLUNA_CONNECTION_TIMEOUT_MS;
}

void USART2_IRQHandler(void)
{
  poll_lidar_uart();
}

static void send_startup_status(void)
{
  static uint8_t report[HID_REPORT_SIZE];
  USBD_HID_HandleTypeDef *hid =
      (USBD_HID_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hid == NULL ||
      hid->state != USBD_HID_IDLE)
  {
    return;
  }

  memset(report, 0, sizeof(report));
  report[0] = 1U;
  report[HID_VERSION_OFFSET] = HID_PROTOCOL_VERSION;
  report[HID_STATUS_OFFSET] = sensor_status;
  USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

static void send_pose_report(bool lidar_fused)
{
  static uint8_t report[HID_REPORT_SIZE];
  USBD_HID_HandleTypeDef *hid =
      (USBD_HID_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[hUsbDeviceFS.classId];
  float quaternion[4];
  float position_m[3];

  /* The USB stack transmits asynchronously, so never overwrite its live buffer. */
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hid == NULL ||
      hid->state != USBD_HID_IDLE)
  {
    return;
  }

  PoseFusion_GetOpenVrPose(&pose_fusion, quaternion, position_m);
  memset(report, 0, sizeof(report));
  report[0] = 1U;
  memcpy(&report[HID_QUATERNION_OFFSET], quaternion, sizeof(quaternion));
  memcpy(&report[HID_POSITION_OFFSET], position_m, sizeof(position_m));
  report[HID_FLAGS_OFFSET] =
      (ENABLE_IMU_DEMO_POSITION ? HID_POSITION_VALID : 0U) |
      (lidar_fused ? HID_LIDAR_FUSED : 0U);
  report[HID_VERSION_OFFSET] = HID_PROTOCOL_VERSION;
  report[HID_STATUS_OFFSET] = sensor_status;
  USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

#if ENABLE_LIDAR_FORWARD_FUSION
static bool update_lidar_forward(uint16_t distance_cm, uint16_t amplitude)
{
  float forward_cosine;
  float lidar_forward_m;
  float correction_error_m;

  if (distance_cm < TFLUNA_MIN_CM || distance_cm > TFLUNA_MAX_CM ||
      amplitude < TFLUNA_MIN_AMPLITUDE || amplitude == 0xFFFFU)
  {
    return false;
  }

  if (lidar_baseline_samples < LIDAR_BASELINE_SAMPLES &&
      !PoseFusion_IsStationary(&pose_fusion))
  {
    lidar_baseline_m = 0.0f;
    lidar_baseline_samples = 0U;
    return false;
  }

  forward_cosine = PoseFusion_ForwardCosine(&pose_fusion);
  if (forward_cosine < LIDAR_MIN_FORWARD_COSINE)
  {
    return false;
  }

  if (lidar_baseline_samples < LIDAR_BASELINE_SAMPLES)
  {
    lidar_baseline_m += 0.01f * distance_cm * forward_cosine;
    lidar_baseline_samples++;
    if (lidar_baseline_samples == LIDAR_BASELINE_SAMPLES)
    {
      lidar_baseline_m /= (float)LIDAR_BASELINE_SAMPLES;
    }
    return false;
  }

  lidar_forward_m = lidar_baseline_m - 0.01f * distance_cm * forward_cosine;
  correction_error_m = fminf(fmaxf(lidar_forward_m - pose_fusion.position_m[0],
                                    -LIDAR_MAX_DISAGREEMENT_M),
                              LIDAR_MAX_DISAGREEMENT_M);
  PoseFusion_CorrectForward(&pose_fusion,
                            pose_fusion.position_m[0] + correction_error_m,
                            LIDAR_CORRECTION_WEIGHT);
  return true;
}
#endif

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
  MX_I2C1_Init();
  init_lidar_uart();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  Icm20948Sample first_sample;
  uint32_t last_imu_ms;
  uint32_t last_hid_ms;
  bool imu_detected;
  bool lidar_ready = false;
#if ENABLE_LIDAR_FORWARD_FUSION
  uint32_t last_lidar_ms;
#endif
  bool lidar_fused = false;

  leds_off();
  sensor_status = 1U; /* Startup wait. */
  /* Both sensors share USB power; allow margin beyond the ICM's 100 ms maximum. */
  HAL_Delay(200U);
  while (1)
  {
    imu_detected = false;
    sensor_status = 2U; /* ICM probe/configuration. */
    if (Icm20948_Init(&imu, &hi2c1))
    {
      imu_detected = true;
      show_sensor_leds(true, false);
      sensor_status = 3U; /* Stationary calibration. */
      if (Icm20948_CalibrateStationary(&imu, GYRO_CALIBRATION_SAMPLES,
                                       IMU_PERIOD_MS))
      {
        sensor_status = 4U; /* First IMU read. */
        if (Icm20948_Read(&imu, &first_sample))
        {
          break;
        }
      }
    }

    uint32_t retry_start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - retry_start_ms) < 1000U)
    {
      lidar_ready = lidar_is_connected(HAL_GetTick());
      sensor_status = lidar_ready ? 8U : 2U;
      if (imu_detected || lidar_ready)
      {
        show_sensor_leds(imu_detected, lidar_ready);
      }
      else
      {
        show_status(LED_R_GPIO_Port, LED_R_Pin);
      }
      send_startup_status();
      HAL_Delay(10U);
    }
    recover_i2c_bus();
  }

  PoseFusion_Init(&pose_fusion, first_sample.accel_g, first_sample.mag_uT,
                  first_sample.mag_valid);
  sensor_status = 5U; /* IMU pose streaming; LiDAR may retry independently. */
#if ENABLE_LIDAR_FORWARD_FUSION
  lidar_ready = lidar_is_connected(HAL_GetTick());
  if (!lidar_ready)
  {
    sensor_status = 7U;
  }
#endif
  last_imu_ms = HAL_GetTick();
  last_hid_ms = last_imu_ms;
#if ENABLE_LIDAR_FORWARD_FUSION
  last_lidar_ms = last_imu_ms;
#endif
  show_sensor_leds(true, lidar_ready);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();
    lidar_ready = lidar_is_connected(now_ms);

    if ((uint32_t)(now_ms - last_imu_ms) >= IMU_PERIOD_MS)
    {
      Icm20948Sample sample;
      float dt_s = (float)(now_ms - last_imu_ms) * 0.001f;
      last_imu_ms = now_ms;
      if (Icm20948_Read(&imu, &sample))
      {
        PoseFusion_Update(&pose_fusion, sample.accel_g, sample.gyro_dps,
                          sample.mag_uT, sample.mag_valid, dt_s);
        if (sensor_status == 6U)
        {
#if ENABLE_LIDAR_FORWARD_FUSION
          sensor_status = lidar_ready ? 5U : 7U;
#else
          sensor_status = 5U;
#endif
        }
        show_sensor_leds(true, lidar_ready);
      }
      else
      {
        sensor_status = 6U; /* Runtime IMU transaction failed. */
        show_status(LED_R_GPIO_Port, LED_R_Pin);
        recover_i2c_bus();
        last_imu_ms = HAL_GetTick();
      }
    }

#if ENABLE_LIDAR_FORWARD_FUSION
    if (lidar_ready && lidar_sample_ready &&
        (uint32_t)(now_ms - last_lidar_ms) >= LIDAR_PERIOD_MS)
    {
      uint16_t distance_cm;
      uint16_t amplitude;

      last_lidar_ms = now_ms;
      HAL_NVIC_DisableIRQ(USART2_IRQn);
      distance_cm = lidar_distance_cm;
      amplitude = lidar_amplitude;
      lidar_sample_ready = false;
      HAL_NVIC_EnableIRQ(USART2_IRQn);
      lidar_fused = update_lidar_forward(distance_cm, amplitude);
      if (sensor_status == 7U)
      {
        sensor_status = 5U;
      }
    }
    else if (!lidar_ready && sensor_status != 6U)
    {
      sensor_status = 7U;
    }
#endif

    if ((uint32_t)(now_ms - last_hid_ms) >= HID_PERIOD_MS)
    {
      last_hid_ms = now_ms;
      send_pose_report(lidar_fused);
      lidar_fused = false;
    }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* Standard mode tolerates the slower edges of both breakouts and their cable. */
  hi2c1.Init.ClockSpeed = 75000;
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
  HAL_GPIO_WritePin(GPIOA, LED_R_Pin|LED_G_Pin|IR_LED_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_CAL_Pin */
  GPIO_InitStruct.Pin = BTN_CAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_CAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin IR_LED_EN_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|IR_LED_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_SPARE1_Pin */
  GPIO_InitStruct.Pin = GPIO_SPARE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_SPARE1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_B_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_SPARE2_Pin */
  GPIO_InitStruct.Pin = GPIO_SPARE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_SPARE2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
