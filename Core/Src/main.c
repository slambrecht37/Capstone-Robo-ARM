/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct Motor {
	int id;
	int min;
	int max;
} motor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for TaskMoveMotor1 */
osThreadId_t TaskMoveMotor1Handle;
const osThreadAttr_t TaskMoveMotor1_attributes = {
  .name = "TaskMoveMotor1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskMoveMotor2 */
osThreadId_t TaskMoveMotor2Handle;
const osThreadAttr_t TaskMoveMotor2_attributes = {
  .name = "TaskMoveMotor2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskMoveMotor3 */
osThreadId_t TaskMoveMotor3Handle;
const osThreadAttr_t TaskMoveMotor3_attributes = {
  .name = "TaskMoveMotor3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskNewPosition */
osThreadId_t TaskNewPositionHandle;
const osThreadAttr_t TaskNewPosition_attributes = {
  .name = "TaskNewPosition",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE BEGIN PV */

int messageAngles1[] = {95, 120, 180, 130, 100, 160};
int messageAngles2[] = {105, 160, 110, 90, 100, 120};

int servoAnglesDefault[] = {110, 100};

int servoAngles2Servos[] = {110, 100}; //arbitrary default values

int messageIndex = 0;

uint8_t uartRxData[10];

int motorPositions[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
void StartMoveMotor1(void *argument);
void StartMoveMotor2(void *argument);
void StartMoveMotor3(void *argument);
void UpdatePosition(void *argument);

/* USER CODE BEGIN PFP */
void servo_write(int);
void servo_sweep(void);
int map(int, int, int, int, int);
int angleToPosition(int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Motor motor1;
struct Motor motor2;
struct Motor motor3;
struct Motor motor4;
struct Motor motor5;
struct Motor motor6;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	motor1.id = 1;
	motor1.max = 160;
	motor1.min = 90;
	motor2.id = 2;
	motor2.max = 155;
	motor2.min = 155;
	motor3.id = 3;
	motor3.max = 254;
	motor3.min = 95;
	motor4.id = 4;
	motor4.max = 253;
	motor4.min = 45;
	motor5.id = 5;
	motor5.max = 253;
	motor5.min = 45;
	motor6.id = 6;
	motor6.max = 253;
	motor6.min = 45;

	uartRxData[0] = 100;
	uartRxData[1] = 100;
	uartRxData[2] = 100;


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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // start the timer
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskMoveMotor1 */
  TaskMoveMotor1Handle = osThreadNew(StartMoveMotor1, NULL, &TaskMoveMotor1_attributes);

  /* creation of TaskMoveMotor2 */
  TaskMoveMotor2Handle = osThreadNew(StartMoveMotor2, NULL, &TaskMoveMotor2_attributes);

  /* creation of TaskMoveMotor3 */
  TaskMoveMotor3Handle = osThreadNew(StartMoveMotor3, NULL, &TaskMoveMotor3_attributes);

  /* creation of TaskNewPosition */
  TaskNewPositionHandle = osThreadNew(UpdatePosition, NULL, &TaskNewPosition_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  HAL_UART_Receive_IT(&huart3, uartRxData, 1);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	  //HAL_Delay(500);
	  //servo_sweep();

	  //only 2, probably first two are valid pulse widths
//	  htim2.Instance->CCR1 = 150;
//	  HAL_Delay(1000);
//	  htim2.Instance->CCR1 = 100;
//	  HAL_Delay(1000);
//	  htim2.Instance->CCR1 = 300;
//	  HAL_Delay(1000);
//	  htim2.Instance->CCR1 = 700;
//	  HAL_Delay(1000);
//	  htim2.Instance->CCR1 = 1000;
//	   HAL_Delay(1000);
	  //htim2.Instance->CCR1 = 1500;
	  //HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 160;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_ODD;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  //do stuff here
  HAL_UART_Receive_IT(&huart3, uartRxData, 1);
}


void servo_write(int angle)
{
	//htim2.Instance->CCR1 = map(0,180,50,250,angle);
	htim2.Instance->CCR1 = angle;
}

void servo_sweep(void)
{
		for(int i = 50; i <= 250; i++)
		{
			servo_write(i);
			HAL_Delay(10);
		}
		for(int i = 250; i >= 50; i--)
		{
			servo_write(i);
			HAL_Delay(10);
		}
}

int map(int st1, int fn1, int st2, int fn2, int value)
{
    return (1.0*(value-st1))/((fn1-st1)*1.0) * (fn2-st2)+st2;
}

int angleToPosition(int angle)
{
	return 188/180 * angle + 65;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMoveMotor1 */
/**
  * @brief  Function implementing the TaskMoveMotor1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMoveMotor1 */
void StartMoveMotor1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	  //osDelay(100); //milliseconds

//	  htim2.Instance->CCR1 = 100;
//	  osDelay(500); //milliseconds
//	  htim2.Instance->CCR1 = 200;
//	  osDelay(500); //milliseconds

	  //htim2.Instance->CCR1 = motorPositions[1];
	  htim2.Instance->CCR1 = 160;

	  osDelay(210);



  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMoveMotor2 */
/**
* @brief Function implementing the TaskMoveMotor2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMoveMotor2 */
void StartMoveMotor2(void *argument)
{
  /* USER CODE BEGIN StartMoveMotor2 */
  /* Infinite loop */
  for(;;)
  {
//	  htim2.Instance->CCR2 = 100;
//	  osDelay(500); //milliseconds
//	  htim2.Instance->CCR2 = 200;
//	  osDelay(500); //milliseconds

	  htim2.Instance->CCR2 = 160;
	  osDelay(220);
  }
  /* USER CODE END StartMoveMotor2 */
}

/* USER CODE BEGIN Header_StartMoveMotor3 */
/**
* @brief Function implementing the TaskMoveMotor3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMoveMotor3 */
void StartMoveMotor3(void *argument)
{
  /* USER CODE BEGIN StartMoveMotor3 */
  /* Infinite loop */
  for(;;)
  {
	  htim2.Instance->CCR3 = motorPositions[0];

    osDelay(200);
  }
  /* USER CODE END StartMoveMotor3 */
}

/* USER CODE BEGIN Header_UpdatePosition */
/**
* @brief Function implementing the TaskNewPosition thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UpdatePosition */
void UpdatePosition(void *argument)
{
  /* USER CODE BEGIN UpdatePosition */
  /* Infinite loop */
  for(;;)
  {
	  //int pos0 = 1;
	  //int pos1 = 1;
	  //int pos2 = 1;
	  int pos0 = angleToPosition(uartRxData[0]);
	  //int pos1 = angleToPosition(uartRxData[1]);
	  //int pos2 = angleToPosition(uartRxData[2]);
	  if (pos0 < motor1.min) {
		  motorPositions[0] = motor1.min;
	  } else if (pos0 > motor1.max) {
		  motorPositions[0] = motor1.max;
	  } else {
		  motorPositions[0] = pos0;
	  }
//	  if (pos1 < motor2.min) {
//	  	  motorPositions[1] = motor2.min;
//	  } else if (pos1 > motor2.max) {
//	  	  motorPositions[1] = motor2.max;
//	  } else {
//	  	  motorPositions[1] = pos1;
//	  }
//	  if (pos2 < motor3.min) {
//	  	  motorPositions[2] = motor3.min;
//	  } else if (pos2 > motor3.max) {
//		  motorPositions[2] = motor3.max;
//	  } else {
//		  motorPositions[2] = pos2;
//  	  }

    osDelay(100);
  }
  /* USER CODE END UpdatePosition */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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

