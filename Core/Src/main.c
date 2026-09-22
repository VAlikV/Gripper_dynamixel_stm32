/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "dynamixel_stm32_ax12_p1.h"
#include <dynamixel_stm32_mx28_p2.h>
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define OPEN 70
#define CLOSE 4090

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
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId dynamixelTaskHandle;

osMutexId dynamixelMutexHandle;

volatile uint8_t usb_cdc_connected = 0;
volatile uint8_t usb_cmd_ready = 0;
volatile uint8_t usb_cmd_buffer[64];
volatile uint32_t usb_cmd_len = 0;

int32_t target_position = OPEN;
int32_t current_position = 0;
int32_t current_velocity = 0;
int32_t current_load = 0;

const uint8_t DXL_ID = 4;
uint8_t command = 1;
uint8_t early_stop = 0;

_Bool opened = 1;

int32_t safety_delta = 25;

uint32_t last_command_tick = 0;
const uint32_t safety_check_delay_ms = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void DynamixelLoop(void const * argument);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  osMutexDef(dynamixelMutex);
  dynamixelMutexHandle = osMutexCreate(osMutex(dynamixelMutex));
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(dynamixelTask, DynamixelLoop, osPriorityNormal, 0, 1024);
  dynamixelTaskHandle = osThreadCreate(osThread(dynamixelTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_pin_GPIO_Port, LED_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_pin_Pin */
  GPIO_InitStruct.Pin = LED_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_pin_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int sign(int a)
{
	if (a > 0) return 1;
	else if (a < 0) return -1;
	else return 0;
}

void usb_print(const char *msg)
{
    uint32_t timeout = HAL_GetTick() + 100;

    if (!usb_cdc_connected)
            return;

	if (CDC_Transmit_FS((uint8_t *)msg, strlen(msg)) == USBD_BUSY)
	{
		return; // просто пропускаем сообщение
	}

//    while (CDC_Transmit_FS((uint8_t *)msg, strlen(msg)) == USBD_BUSY)
//    {
//        if (HAL_GetTick() > timeout)
//            break;
//    }
}

void process_usb_command(const char *cmd)
{

    if (strcmp(cmd, "Open") == 0)
    {
//    	usb_print_blocking("Opening...\r\n");
    	if (!opened)
    	{
    		osMutexWait(dynamixelMutexHandle, osWaitForever);
			target_position = OPEN;
			last_command_tick = HAL_GetTick();
			command = 1;
			early_stop = 0;
			osMutexRelease(dynamixelMutexHandle);
			opened = 1;
    	}

    }
    else if (strcmp(cmd, "Close") == 0)
    {
//    	usb_print_blocking("Closing...\r\n");
    	if (opened)
		{
			osMutexWait(dynamixelMutexHandle, osWaitForever);
			target_position = CLOSE;
			last_command_tick = HAL_GetTick();
			command = 1;
			early_stop = 0;
			osMutexRelease(dynamixelMutexHandle);
			opened = 0;
		}
    }
    else
    {
        usb_print("Unknown command\r\n");
    }
}

void DynamixelStartUp()
{
	dynamixel_set_led(&huart1, DXL_ID, 1);
	osDelay(300);
	dynamixel_set_led(&huart1, DXL_ID, 0);
	osDelay(300);

	dynamixel_set_torque_enable(&huart1, DXL_ID, 0);
	osDelay(100);

	dynamixel_set_operating_mode(&huart1, DXL_ID, DXL_MODE_POSITION);
	osDelay(100);

//	dynamixel_set_profile_acceleration(&huart1, DXL_ID, 1024);
//	osDelay(20);
//
//	dynamixel_set_profile_velocity(&huart1, DXL_ID, 1024);
//	osDelay(20);

//	dynamixel_set_joint_mode(&huart1, DXL_ID);
	dynamixel_set_p_value(&huart1, DXL_ID, 100);
	osDelay(20);
	dynamixel_set_d_value(&huart1, DXL_ID, 10);
	osDelay(20);

	dynamixel_set_torque_enable(&huart1, DXL_ID, 1);
	osDelay(300);
}

void DynamixelLoop(void const * argument)
{
    osDelay(1000);

    osMutexWait(dynamixelMutexHandle, osWaitForever);
    DynamixelStartUp();
    last_command_tick = HAL_GetTick();
    osMutexRelease(dynamixelMutexHandle);

    int32_t local_target = 0;
	uint32_t local_last_command_tick = 0;

    for (;;)
    {

        osMutexWait(dynamixelMutexHandle, osWaitForever);

        local_target = target_position;
        local_last_command_tick = last_command_tick;

        if (command > 0)
        {
        	dynamixel_set_goal_position(&huart1, DXL_ID, local_target);
        	command = 0;
        }

        dynamixel_read_present_position(&huart1, DXL_ID, &current_position);
        dynamixel_read_present_position(&huart1, DXL_ID, &current_position);

        dynamixel_read_present_moving_velocity(&huart1, DXL_ID, &current_velocity);
//        dynamixel_read_present_moving_velocity(&huart1, DXL _ID, &current_velocity);

        dynamixel_read_present_load(&huart1, DXL_ID, &current_load);
//        dynamixel_read_present_load(&huart1, DXL_ID, &current_load);

        osMutexRelease(dynamixelMutexHandle);

        uint32_t now = HAL_GetTick();

        if ((early_stop != 1) && (now - local_last_command_tick) >= safety_check_delay_ms)
        {
            if ((abs(current_velocity) < 50) &&
                (abs(current_position - local_target) >= safety_delta))
            {
                osMutexWait(dynamixelMutexHandle, osWaitForever);

                target_position =
                    current_position +
                    safety_delta * sign(current_load);

                last_command_tick = HAL_GetTick();

                command = 1;
                early_stop = 1;

                osMutexRelease(dynamixelMutexHandle);
            }
        }

        osDelay(1);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  MX_USB_DEVICE_Init();

  osDelay(2000);

  char msg[96];

  uint32_t last_telemetry_tick = 0;

  for (;;)
  {

    if (usb_cmd_ready)
    {
        char cmd[64];

        __disable_irq();

        uint32_t len = usb_cmd_len;
        if (len > 63)
            len = 63;

        for (uint32_t i = 0; i <= len; i++)
        {
            cmd[i] = usb_cmd_buffer[i];
        }

        usb_cmd_ready = 0;

        __enable_irq();

        for (uint32_t i = 0; i < len; i++)
        {
            if (cmd[i] == '\r' || cmd[i] == '\n')
            {
                cmd[i] = '\0';
                break;
            }
        }

        process_usb_command(cmd);
    }

    if (usb_cdc_connected && (HAL_GetTick() - last_telemetry_tick >= 50))
    {
        last_telemetry_tick = HAL_GetTick();

        snprintf(
            msg,
            sizeof(msg),
            "target=%ld pos=%ld vel=%ld load=%ld\r\n",
            (long)target_position,
            (long)current_position,
            (long)current_velocity,
            (long)current_load
        );

        usb_print(msg);
    }

    osDelay(5);
  }
}
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
