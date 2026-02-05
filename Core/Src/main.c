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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "esp01s.h"
#include "flight_control.h"
#include "motor.h"
#include "mpu6050.h"
#include "imu.h"
#include "mahony.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
IMU_Data_t imu;
Attitude_t att;
volatile uint8_t imu_attitude_ready_flag = 0;
ESP01S_Status_t status;
// UART 中断接收字节
uint8_t rx1_byte, rx2_byte;          // 单字节接收缓存
uint8_t uart2_buf[256];   // 缓存 PC 发来的数据
uint16_t uart2_idx = 0;              // 缓冲区索引
int global_step = 0;  // 定义
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);
  HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  // HAL_TIM_Base_Start_IT(&htim2);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // 初始化ESP01S
  status = ESP01S_Init(&huart1, 2000);
  status = ESP01S_ConnectWiFi();
  status = ESP01S_ConnectTCP();

  // if (MPU6050_Init() == 0)
  // {
  //   while (1) {
  //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  //     HAL_Delay(100);
  //   }
  // }
  // IMU_Init();
  // Mahony_Init(1.0f, 0.0f);  // Kp=1.0, Ki=0
  Motor_Init();
  Motor_Arm();     // 上电解锁 3 秒
  // FlightControl_Init();
  // FlightControl_Arm();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(ESP01S_IRQ_DataReady())
    {
      ControlInput_t ci = ESP01S_IRQ_GetControlInput();
      FlightControl_SetInput(&ci);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      if (ci.btn_yaw_cw == 1) {
        Motor_SetAll(1100);
      } else if (ci.btn_yaw_ccw == 1) {
        Motor_SetAll(1200);
      } else if (ci.btn_up == 1) {
        Motor_SetAll(1400);
      } else if (ci.btn_down == 1) {
        Motor_SetAll(1300);
      } else {
        Motor_SetAll(1000);
      }
    }
    // if (imu_attitude_ready_flag) {
    //   imu_attitude_ready_flag = 0;
    //   // MotorControl_Update(&att, target_roll, target_pitch, target_yaw); // 更新电机 PWM，已由飞行控制模块接管
    //   FlightControl_Update(&att);
    // }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
    IMU_Update(&imu); // 每 2ms 调用一次
    Mahony_Update(&imu, 0.002f, &att); // dt = 2ms
    imu_attitude_ready_flag = 1;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    if (rx1_byte == '>')
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    HAL_UART_Transmit_IT(&huart2, &rx1_byte, 1);
    ESP01S_IRQ_ProcessByte(rx1_byte);
    HAL_UART_Receive_IT(&huart1, &rx1_byte, 1);

  }
  else if(huart->Instance == USART2)
  {
    // 缓存数据
    uart2_buf[uart2_idx++] = rx2_byte;

    // 检查是否遇到 \r 或 \n
    if(rx2_byte == '\n' || uart2_idx >= 256)
    {
      if(uart2_idx > 0)
      {
        // 发送整段缓存到 UART1
        HAL_UART_Transmit_IT(&huart1, uart2_buf, uart2_idx);
        uart2_idx = 0; // 重置缓存索引
      }
    }
    // 再次接收
    HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
  }
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
