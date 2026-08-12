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
#define TFLUNA_I2C_ADDRESS         (0x10U << 1)
#define TFLUNA_DISTANCE_REG        0x00U
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
#if ENABLE_LIDAR_FORWARD_FUSION
static float lidar_baseline_m;
static uint8_t lidar_baseline_samples;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void leds_off(void);
static void show_status(GPIO_TypeDef *port, uint16_t pin);
static void send_pose_report(bool lidar_fused);
#if ENABLE_LIDAR_FORWARD_FUSION
static bool update_lidar_forward(void);
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
  USBD_HID_SendReport(&hUsbDeviceFS, report, sizeof(report));
}

#if ENABLE_LIDAR_FORWARD_FUSION
static bool update_lidar_forward(void)
{
  uint8_t data[4];
  uint16_t distance_cm;
  uint16_t amplitude;
  float forward_cosine;
  float lidar_forward_m;
  float correction_error_m;

  if (lidar_baseline_samples < LIDAR_BASELINE_SAMPLES &&
      !PoseFusion_IsStationary(&pose_fusion))
  {
    lidar_baseline_m = 0.0f;
    lidar_baseline_samples = 0U;
    return false;
  }

  if (HAL_I2C_Mem_Read(&hi2c1, TFLUNA_I2C_ADDRESS, TFLUNA_DISTANCE_REG,
                       I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 10U) != HAL_OK)
  {
    return false;
  }

  distance_cm = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  amplitude = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
  if (distance_cm < TFLUNA_MIN_CM || distance_cm > TFLUNA_MAX_CM ||
      amplitude < TFLUNA_MIN_AMPLITUDE || amplitude == 0xFFFFU)
  {
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  Icm20948Sample first_sample;
  uint32_t last_imu_ms;
  uint32_t last_hid_ms;
#if ENABLE_LIDAR_FORWARD_FUSION
  uint32_t last_lidar_ms;
#endif
  bool lidar_fused = false;

  show_status(LED_B_GPIO_Port, LED_B_Pin);
  HAL_Delay(100U); /* ICM-20948 registers may be unavailable for 100 ms after power-up. */
  while (!Icm20948_Init(&imu, &hi2c1) ||
         !Icm20948_CalibrateStationary(&imu, GYRO_CALIBRATION_SAMPLES,
                                       IMU_PERIOD_MS) ||
         !Icm20948_Read(&imu, &first_sample))
  {
    show_status(LED_R_GPIO_Port, LED_R_Pin);
    HAL_Delay(1000U);
    show_status(LED_B_GPIO_Port, LED_B_Pin);
  }

  PoseFusion_Init(&pose_fusion, first_sample.accel_g, first_sample.mag_uT,
                  first_sample.mag_valid);
  last_imu_ms = HAL_GetTick();
  last_hid_ms = last_imu_ms;
#if ENABLE_LIDAR_FORWARD_FUSION
  last_lidar_ms = last_imu_ms;
#endif
  show_status(imu.mag_available ? LED_G_GPIO_Port : LED_B_GPIO_Port,
              imu.mag_available ? LED_G_Pin : LED_B_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_imu_ms) >= IMU_PERIOD_MS)
    {
      Icm20948Sample sample;
      float dt_s = (float)(now_ms - last_imu_ms) * 0.001f;
      last_imu_ms = now_ms;
      if (Icm20948_Read(&imu, &sample))
      {
        PoseFusion_Update(&pose_fusion, sample.accel_g, sample.gyro_dps,
                          sample.mag_uT, sample.mag_valid, dt_s);
        show_status(imu.mag_available ? LED_G_GPIO_Port : LED_B_GPIO_Port,
                    imu.mag_available ? LED_G_Pin : LED_B_Pin);
      }
      else
      {
        show_status(LED_R_GPIO_Port, LED_R_Pin);
      }
    }

#if ENABLE_LIDAR_FORWARD_FUSION
    if ((uint32_t)(now_ms - last_lidar_ms) >= LIDAR_PERIOD_MS)
    {
      last_lidar_ms = now_ms;
      lidar_fused = update_lidar_forward();
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
  hi2c1.Init.ClockSpeed = 400000;
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
