/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_x-cube-ble1.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <cstdio>
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
UART_HandleTypeDef huart3;
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId askForDataTaskHandle;
osThreadId presentationTaskHandle;
osThreadId communicationTaskHandle;
osMutexId uartMutexHandle;
osMutexId newDataMutexHandle;
xQueueHandle msgQueueHandle;
uint16_t counter;
uint8_t sensorObjectCount;
uint8_t whichSensorWrites;
uint8_t whichLoopIteration;
uint8_t sentConfigurationMsg[20];
uint8_t newData;
char uartData[70];
bool newConfig;
extern volatile bool all_servers_connected;
extern volatile bool client_ready;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void AskForDataTaskThread(void const * argument);
void PresentationTaskThread(void const * argument);
void CommunicationTaskThread(void const * argument);
void presentDataFromSensor(uint8_t which);
void delayMicroseconds(uint32_t us);
void prepareNewConfig(uint8_t sensorType, uint16_t interval, uint8_t *name);
bool checkIfTempSensorReadoutCorrect(uint32_t dataBits, uint8_t checksumBits);
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
  MX_USART3_UART_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  uartMutexHandle = xSemaphoreCreateMutex();
  newDataMutexHandle = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  msgQueueHandle = xQueueCreate(MAX_MSGS, sizeof(uartData));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128/*256*/);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(askForDataTask, AskForDataTaskThread, osPriorityNormal, 0, 128/*256*/);
	askForDataTaskHandle = osThreadCreate(osThread(askForDataTask), NULL);

	osThreadDef(presentationTask, PresentationTaskThread, osPriorityNormal, 0, /*128*/512);
	presentationTaskHandle = osThreadCreate(osThread(presentationTask), NULL);

	osThreadDef(communicationTask, CommunicationTaskThread, osPriorityLow, 0, 128/*256*/);
	communicationTaskHandle = osThreadCreate(osThread(communicationTask), NULL);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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

  //dla RPi
  huart3.Init.BaudRate = 115200;
//  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(client_ready){
		  //Wyslij sygnal do taska odczytu ze powinien teraz sie uruchomic
		  xTaskNotify(askForDataTaskHandle, 0x01, eSetBits);
	  }
	  else {
	  	  MX_BlueNRG_MS_Process();
	  }

	  //
	  //osDelay(DELAY_TIME/30); //wlaczenie delaya na 100 milisek. powoduje wypisywanie smieci w terminalu - czemu?

  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN 6 */
void AskForDataTaskThread(void const * argument)
{
	/* Podejscie: trzy taski - supervisor -> odczyt -> prezentacja: task odczytu danych */
	uint32_t notifValue;
	/* Infinite loop */
	for(;;)
	{
		xTaskNotifyWait(pdFALSE, 0xFF, &notifValue, portMAX_DELAY);
		if((notifValue&0x01) != 0x00) //Sprawdza czy notifValue zawiera wartosc ktora wyslal task supervisora
		{
		  //prymitywne wyslanie danych konfiguracji (docelowo bedzie do tego interfejs). chyba juz tu powinno byc wskazane do ktorego servera ma trafic konf.
		  //!zwrocic uwage na delay glownego taska i wartosc countera!
		  if(counter == /*0*/UINT16_MAX/16){
			  prepareNewConfig(DHT22, 4, (uint8_t *)"Pokoj");
		  }
		  if(counter == /*4*/UINT16_MAX/8){
			  prepareNewConfig(DHT22, 5, (uint8_t *)"Kuchnia");
		  }
		  //uwazac zeby tylko raz wysylac pozadana konfiguracje - a nie w petli co przepelnienie wartosci countera!
		  if(counter <= UINT16_MAX/8){
			  counter++;
		  }

	      MX_BlueNRG_MS_Process();

	      //!
	      xSemaphoreTake(newDataMutexHandle, DELAY_TIME);
	      if(newData){ //uruchom task prezentacji tylko wtedy, gdy przyjda nowe dane
	       	  //Wyslij sygnal do taska od prezentacji ze powinien teraz sie uruchomic
	    	  xTaskNotify(presentationTaskHandle, 0x02, eSetBits);
	      }
	      xSemaphoreGive(newDataMutexHandle);
		}
	}
}

void PresentationTaskThread(void const * argument)
{
	/* Podejscie: trzy taski - supervisor -> odczyt -> prezentacja: task prezentacji danych */
	uint32_t notifValue;
	/* Infinite loop */
	for(;;)
	{
		//Czekaj na sygnal od taska odczytu
		xTaskNotifyWait(pdFALSE, 0xFF, &notifValue, portMAX_DELAY);
		if((notifValue&0x02) != 0x00) //Sprawdza czy notifValue zawiera wartosc ktora wyslal task odczytu
		{
			//Format: nazwa_czujnika '\0' dane
			char name[MAX_NAME_LEN]; int i;
			memset(name, 0x00, sizeof(name));
			//
			xSemaphoreTake(newDataMutexHandle, DELAY_TIME);
			while(newData){ //problem klienta byl z synchronizacja wartosci tej zmiennej?
				newData--;
				for(i=0; dataBLE[newData][i] != '\0' && i<MAX_NAME_LEN; i++){
					name[i] = dataBLE[newData][i];
				}
				uint32_t dataBits = (dataBLE[newData][i+1] << 24) + (dataBLE[newData][i+2] << 16)
								  + (dataBLE[newData][i+3] << 8)  + (dataBLE[newData][i+4]);
				uint8_t checksumBits = dataBLE[newData][i+5];
				if(checkIfTempSensorReadoutCorrect(dataBits, checksumBits)){
					uint16_t humid = (dataBLE[newData][i+1] << 8) | dataBLE[newData][i+2];
					uint16_t temp  = (dataBLE[newData][i+3] << 8) | dataBLE[newData][i+4];
					uint16_t humidDecimal = humid%10;
					uint16_t tempDecimal  = temp%10;
					temp = temp/(uint16_t)10;
					humid= humid/(uint16_t)10;
					//xQueueSend(msgQueueHandle, (uint8_t *)uartData, 100);
					printf("\r\nCzujnik %s\r\n", name);

					memset(uartData, 0x0, sizeof(uartData));
					sprintf(uartData, "\r\nCzujnik %s\r\n", name);
					HAL_UART_Transmit(&huart3, (uint8_t *)uartData,
							sizeof("\r\nCzujnik %s\r\n")+sizeof(name), 10);

					printf("Temperatura\t %hu.%huC\r\nWilgotnosc\t %hu.%hu%%\r\n",
							  temp, tempDecimal, humid, humidDecimal);

//					memset(uartData, 0x0, sizeof(uartData));
					sprintf(uartData, "Temperatura\t %hu.%huC\r\n", temp, tempDecimal);
					HAL_UART_Transmit(&huart3, (uint8_t *)uartData,
							sizeof("Temperatura\t %hu.%huC\r\n")+2*sizeof(uint16_t), 10);

//					memset(uartData, 0x0, sizeof(uartData));
					sprintf(uartData, "Wilgotnosc\t %hu.%hu%%\r\n", humid, humidDecimal);
					HAL_UART_Transmit(&huart3, (uint8_t *)uartData,
							sizeof("Wilgotnosc\t %hu.%hu%%\r\n")+2*sizeof(uint16_t), 10);
				}
			}
			xSemaphoreGive(newDataMutexHandle);
		}
	}
}

//
void CommunicationTaskThread(void const * argument)
{
	char receivedData[50];
	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(msgQueueHandle, receivedData, DELAY_TIME); //delayTime?
		xSemaphoreTake(uartMutexHandle, DELAY_TIME); //delayTime?
		printf(receivedData);
		xSemaphoreGive(uartMutexHandle);
	}
}
/* USER CODE END 6 */

/* USER CODE BEGIN 7 */
void delayMicroseconds(uint32_t us){
	//Average, experimental time for 1 rotation of the 'for' loop with nops: ~140ns
	//for an 80MHz processor@max speed; that gives ~7.143 loop rotations for 1 ms
	//Use this fact and the processor frequency to adjust the loop counter value for any processor speed
	uint32_t clockFreq = HAL_RCC_GetHCLKFreq();	//Current processor frequency
	float clockFreqRel = clockFreq/(float)80000000.0;//Current processor frequency relative to base of 80MHz
	uint32_t loopCounter = (us > 0 ? (uint32_t)(us*clockFreqRel*7.143) : (uint32_t)(clockFreqRel*7.143));
	//uint32_t loopCounter = (us > 0 ? (uint32_t)(us*7.143) : 7); //A minimum delay of 1 us - 80MHz only
	for(uint32_t tmp = 0; tmp < loopCounter; tmp++) {asm("nop");}
	//previously there was tmp < 800 giving 3200 processor cycles, each lasting 12.5 ns = 40 us delay
	//UINT_MAX	Maximum value for a variable of type unsigned int	4,294,967,295 (0xffffffff)
}

void prepareNewConfig(uint8_t sensorType, uint16_t interval, uint8_t *name){
	/* Format wiadomosci: <typ_sensora:1B> <interwal:2B> <nazwa:max.14B> */
	for(int i=0; i<MSG_LEN; i++){
		sentConfigurationMsg[i] = '\0';
	}
	sentConfigurationMsg[0] = sensorType;
	sentConfigurationMsg[1] = interval/256;
	sentConfigurationMsg[2] = interval%256;
	int i = 0;
	for(i=3; name[i-3] != '\0' && i<MSG_LEN; i++){
		sentConfigurationMsg[i] = name[i-3];
	}
	if(i < MSG_LEN){
		sentConfigurationMsg[i] = '\0';
	}
	newConfig = true;
}

bool checkIfTempSensorReadoutCorrect(uint32_t dataBits, uint8_t checksumBits){
	uint8_t value = ((dataBits >> 24) & 0xFF) + ((dataBits >> 16) & 0xFF) + ((dataBits >> 8) & 0xFF) + (dataBits & 0xFF);
	if(value == checksumBits)
		return true;
	return false;
}
/* USER CODE END 7 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
