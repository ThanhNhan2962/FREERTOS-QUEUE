/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
/* USER CODE BEGIN Includes */
#define Led_Pin GPIO_Pin_1
#define Button_Pin GPIO_Pin_0
#define Led_Port GPIO_Port_1
#define Led_Port GPIO_Pport_1
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef enem {
	Led_On,
	Led_Off,
	Led_Toggle
} led_command_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static void LED_Task( void *pvParameters );
static void SW_Task( void *pvParameters );
static void UART_Task( void *pvParameters );
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t   status_task_Handle;
xQueueHandle	queue_led_command;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  UARTStdioConfig(USART1,USART1_IRQn, true);
  queue_led_command = xQueueCreate(5, sizeof(led_command_t));

  xTaskCreate(LED_Task, "LED Task", 128, NULL, 1, &TaskHandle_LED);
  xTaskCreate(SW_Task, "Switch Task", 128, NULL, 1, &TaskHandle_SW);
  xTaskCreate(UART_Task, "UART Task", 128, NULL, 1, &TaskHandle_UART);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  vTaskStartScheduler();

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(16000000);
  LL_SetSystemCoreClock(16000000);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2|LL_GPIO_PIN_3);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void LED_Task(void *pvParameters) {
    LedCommand_t cmd;
    while(1) {
        if (xQueueReceive(Queue_LED_Commands, &cmd, portMAX_DELAY) == pdPASS) {
            switch(cmd) {
                case LED_ON:
                    LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
                    break;
                case LED_OFF:
                    LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
                    break;
                case LED_TOGGLE:
                    LL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
                    break;
            }
        }
    }
}

static void SW_Task(void *pvParameters) {
    uint32_t current_state, last_state = RELEASED;
    while(1) {
        current_state = read_button(); // Use your existing read_button function
        if (current_state != last_state) {
            last_state = current_state;
            if (current_state == PRESSED) {
                xQueueSend(Queue_LED_Commands, (void *) &LED_TOGGLE, portMAX_DELAY);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Debounce delay
    }
}

static void UART_Task(void *pvParameters) {
    char rx_buffer[10];
    while(1) {
        // Implement UART receive logic here. Use HAL_UART_Receive_IT or similar non-blocking API.
        // Simulated receive function:
        if (Receive_UART_Command(rx_buffer)) { // You need to implement this
            if (strncmp(rx_buffer, "ON\r\n", 5) == 0) {
                LedCommand_t cmd = LED_ON;
                xQueueSend(Queue_LED_Commands, &cmd, 0);
            } else if (strncmp(rx_buffer, "OFF\r\n", 6) == 0) {
                LedCommand_t cmd = LED_OFF;
                xQueueSend(Queue_LED_Commands, &cmd, 0);
            } else if (strncmp(rx_buffer, "TOGGLE\r\n", 8) == 0) {
                LedCommand_t cmd = LED_TOGGLE;
                xQueueSend(Queue_LED_Commands, &cmd, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust as necessary for your context
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
