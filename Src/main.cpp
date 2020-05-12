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
typedef enum{
	ONE_OUTPUT = 0,
	ONE_INPUT = 1,
} oneWireMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Bit fields manipulations
#define bitRead(value, bit)		(((value) >> (bit)) & 0x01)
#define bitSet(value, bit)		((value) |= (1UL << (bit)))
#define bitClear(value, bit)	((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
//Port for one-wire communication
#define oneWirePort	((GPIO_TypeDef *) GPIOA_BASE) //GPIOA
//Delay time between following reads in ms
#define delayTime	3000
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
xQueueHandle msgQueueHandle;
uint8_t sensorObjectCount;
uint8_t whichSensorWrites;
uint8_t whichLoopIteration;
char uartData[50];

//Bez DM-a i Slave'ow
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
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  msgQueueHandle = xQueueCreate(5, sizeof(uartData));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(askForDataTask, AskForDataTaskThread, osPriorityNormal, 0, 128);
	askForDataTaskHandle = osThreadCreate(osThread(askForDataTask), NULL);

	osThreadDef(presentationTask, PresentationTaskThread, osPriorityNormal, 0, 128);
	presentationTaskHandle = osThreadCreate(osThread(presentationTask), NULL);

	osThreadDef(communicationTask, CommunicationTaskThread, osPriorityLow, 0, 128);
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
  huart3.Init.BaudRate = 115200;
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
	  if(whichLoopIteration++ > 7){
		  //Wyslij sygnal do taska odczytu ze powinien teraz sie uruchomic
		  xTaskNotify(askForDataTaskHandle, 0x01, eSetBits);
	  }
	  else {
	  	  MX_BlueNRG_MS_Process();
	  }
	  osDelay(3000);
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
		  //Na razie wersja tylko z jednym serverem
		  //vTaskDelay(150); //moze byc niebezpieczne!
		  delayMicroseconds(150000);
	      MX_BlueNRG_MS_Process();
		  //Wyslij sygnal do taska od prezentacji ze powinien teraz sie uruchomic
		  xTaskNotify(presentationTaskHandle, 0x02, eSetBits);
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
			//Na razie wersja tylko z jednym serverem
			uint16_t humid = (dataBLE[0] << 8) | dataBLE[1];
			uint16_t temp  = (dataBLE[2] << 8) | dataBLE[3];
			uint16_t humidDecimal = humid%10;
			uint16_t tempDecimal  = temp%10;
			temp = temp/(uint16_t)10;
			humid= humid/(uint16_t)10;
			//xQueueSend(msgQueueHandle, (uint8_t *)uartData, 100);
			//!!!!
			printf("\r\nTemperatura\t %hu.%huC\r\nWilgotnosc\t %hu.%hu%%\r\n",
					  temp, tempDecimal, humid, humidDecimal);
		}
	}
}

void CommunicationTaskThread(void const * argument)
{
	char receivedData[50];
	/* Infinite loop */
	for(;;)
	{
		xQueueReceive(msgQueueHandle, receivedData, delayTime); // delayTime???
		xSemaphoreTake(uartMutexHandle, 1000); // 1000???
//		HAL_UART_Transmit(&huart3, (uint8_t *)receivedData, strlen(receivedData), 10);
		printf(receivedData);
		xSemaphoreGive(uartMutexHandle);
		//vTaskDelay(1000); // reschedule - a moze sygnal miedzy zadaniami?
	}
}
/* USER CODE END 6 */

/* USER CODE BEGIN 7 */
//DeviceManager
//DeviceManager::DeviceManager(){
//	occupiedPinsCount = 0;
////	hardwarePinsList[0] = ((uint16_t)0x0001); /* Pin PA0 selected */ //EXTI line
////	hardwarePinsList[1] = ((uint16_t)0x0002); /* Pin PA1 selected */ //CS line
////	hardwarePinsList[2] = ((uint16_t)0x0004); /* Pin PA2 selected */ //USART2 TX
////	hardwarePinsList[3] = ((uint16_t)0x0008); /* Pin PA3 selected */ //USART2 RX
//	hardwarePinsList[0] = ((uint16_t)0x0010); /* Pin PA4 selected */
////	hardwarePinsList[1] = ((uint16_t)0x0020); /* Pin PA5 selected */ //LED
////	hardwarePinsList[2] = ((uint16_t)0x0040); /* Pin PA6 selected */ //SPI MISO
////	hardwarePinsList[3] = ((uint16_t)0x0080); /* Pin PA7 selected */ //SPI MOSI
////	hardwarePinsList[4] = ((uint16_t)0x0100); /* Pin PA8 selected */ //Reset line
//	hardwarePinsList[1] = ((uint16_t)0x0200); /* Pin PA9 selected */
//	hardwarePinsList[2] = ((uint16_t)0x0400); /* Pin PA10 selected*/
//}
//uint16_t DeviceManager::getNewCommunicationPin(void){
//	if(occupiedPinsCount < 2){
//		return hardwarePinsList[occupiedPinsCount++];
//	}
//	else {
//		return 0xFFFF;
//	}
//}
//uint16_t DeviceManager::getCommunicationPinAddress(int index){
//	if(index >= 0 && index <= occupiedPinsCount){
//		return hardwarePinsList[index];
//	}
//	else{
//		return 0xFFFF;
//	}
//}
//
//uint8_t DeviceManager::getOccupiedPinsCount(void){
//	sprintf(uartData, "\r\nOccupied pins count: %hu", occupiedPinsCount);
//	xQueueSend(msgQueueHandle, (uint8_t *)uartData, 100);
//	return occupiedPinsCount;
//}
//
////Sensor
//Sensor::Sensor(DeviceManager &dm){
//	communicationPort = oneWirePort;
//	communicationPin = dm.getNewCommunicationPin(); /// !!!
//	for(uint8_t i=0; i<16; i++){
//		if(communicationPin & (1 << i)){
//			communicationPinID = i;
//			break;
//		}
//	}
//	sensorObjectIndex = sensorObjectCount++;
//}
//
//uint16_t Sensor::getCommunicationPin(void){
//	return this->communicationPin;
//}
//void Sensor::printCommunicationPin(void){
//	sprintf(uartData, "\r\nCommunication pin: %hu", this->communicationPin);
//	xQueueSend(msgQueueHandle, (uint8_t *)uartData, 100);
//}
//void Sensor::changePinMode(oneWireMode mode){
//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.Pin = this->communicationPin;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	if(mode == ONE_OUTPUT){
//		//oneWire_PORT->MODER &= ~(3UL << 2*oneWirePin_Idx); //Reset State
//		//oneWire_PORT->MODER |= (0x01 << 2*oneWirePin_Idx); //Output Mode
//		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	}
//	else if(mode == ONE_INPUT){
//		//oneWire_PORT->MODER &= ~(3UL << 2*oneWirePin_Idx); //Input Mode
//		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	}
//	HAL_GPIO_Init(oneWirePort, &GPIO_InitStruct);
//}
//
//void Sensor::writePin(bool state){
//	if(state){
//		HAL_GPIO_WritePin(oneWirePort, this->communicationPin, GPIO_PIN_SET);
//	}
//	else{
//		HAL_GPIO_WritePin(oneWirePort, this->communicationPin, GPIO_PIN_RESET);
//	}
//}
//
//bool Sensor::readPin(void){
//	return (1&HAL_GPIO_ReadPin(oneWirePort, this->communicationPin));
//}

//Method performing reading temperature and humidity data from sensor
//void Sensor::performReadFromSensor(){
//	this->changePinMode(ONE_OUTPUT);	//Change data pin mode to OUTPUT
//	this->writePin(0);					//Put pin LOW
//	vTaskDelay(1 / portTICK_PERIOD_MS);	//Delay for 1 ms to let the sensor wake up
//	this->writePin(1);					//Bring pin HIGH
//	delayMicroseconds(20);				//Wait a bit until the high state on the pin is set
//	this->changePinMode(ONE_INPUT);		//Set pin as input
//	uint32_t rawBits = 0UL;
//	uint8_t checksumBits = 0;
//	//Receive the alternating states to confirm the sensor is about to send data
//	while(this->readPin());
//	while(!this->readPin());
//	while(this->readPin());
//	for(int8_t i = 31; i>=0; i--)		//Read 32 bits of temp.&humidity data
//	{
//		/*
//		* Bit data "0" signal: the level is LOW for 50ms and HIGH for 26-28ms;
//		* Bit data "1" signal: the level is LOW for 50ms and HIGH for 70ms;
//		* MAX FREQUENCY ON STM32L476RG = 80MHz
//		* SO IT TAKES 12,5 ns FOR ONE INSTRUCTION TO EXECUTE
//		* A DELAY OF 1 SECOND (x TICKS): 80 MILLION NOP INSTRUCTIONS TO EXECUTE
//		*/
//	  while(!this->readPin());
//	  //Experimentally determined average time of one rotation of the 'for' loop with nop: 140 ns
//	  //A correct value range for 26/28 to 70 us delay is then <190; 500>
//	  delayMicroseconds(50);
//	  if(this->readPin())
//	  {
//		  rawBits |= (1UL << i);
//	  }
//	  while(this->readPin());
//	}
//	for(int8_t i = 7; i>=0; i--)		//Read 8 bits of checksum data
//	{
//	  while(!this->readPin());
//	  delayMicroseconds(50);
//	  if(this->readPin())
//	  {
//		  checksumBits |= (1UL << i);
//	  }
//	  while(this->readPin());
//	}
//	data[0] = (rawBits>>24)&0xFF;		//Copy raw data bits to an array of bytes
//	data[1] = (rawBits>>16)&0xFF;
//	data[2] = (rawBits>>8)&0xFF;
//	data[3] = (rawBits>>0)&0xFF;
//	data[4] = (checksumBits)&0xFF;
//}

//void presentDataFromSensor(uint8_t which){
//	//Calculate the temperature and humidity values
//	uint8_t myChecksum;
//	uint16_t temp, humidity, tempDecimal, humidityDecimal, temp16, humid16;
//	uint16_t ten = (uint16_t)10;
//	//Calculate checksum
//	myChecksum = 0;
//	for(uint8_t i=0; i<4; i++)
//	{
//	  myChecksum += data[i];
//	}
//	if(myChecksum == data[4])
//	{
//	  temp16 = (data[2] << 8) | data[3];
//	  humid16= (data[0] << 8) | data[1];
//	  temp = temp16/ten;
//	  tempDecimal = temp16%ten;
//	  humidity = humid16/ten;
//	  humidityDecimal = humid16%ten;
//
////	  sprintf(uartData, "\r\nServer %hu:", which+1);
////	  //xQueueSend(msgQueueHandle, (uint8_t *)uartData, 100);	//100 ticks to wait???
////	  HAL_UART_Transmit(&huart3, (uint8_t *)uartData, sizeof(uartData), 100);
////	  sprintf(uartData, "\r\nTemperatura\t %hu.%huC"
////			  "\r\nWilgotnosc\t %hu.%hu%%\r\n",
////			  temp, tempDecimal, humidity, humidityDecimal);
////	  //xQueueSend(msgQueueHandle, (uint8_t *)uartData, 100); //100???
////	  HAL_UART_Transmit(&huart3, (uint8_t *)uartData, sizeof(uartData), 100);
//
//	  printf("Temperatura: \r\n");
//	}
//	else
//	{
//	  strcpy(uartData, "\r\nBlad odczytu danych z czujnika!");
//	  //xQueueSend(msgQueueHandle, (uint8_t *)uartData, 100);
//	  HAL_UART_Transmit(&huart3,(uint8_t *) uartData, sizeof(uartData), 100);
//	}
//}

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
