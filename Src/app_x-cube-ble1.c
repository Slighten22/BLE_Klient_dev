/**
  ******************************************************************************
  * File Name          : app_x-cube-ble1.c
  * Description        : Implementation file
  *             
  ******************************************************************************
  *
  * COPYRIGHT 2020 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_X_CUBE_BLE1_C
#define __APP_X_CUBE_BLE1_C
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_x-cube-ble1.h"

#include "hci_tl.h"
#include "sample_service.h"
#include "role_type.h"
#include "bluenrg_utils.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"

/* USER CODE BEGIN */
#include <string.h>
#include <stdbool.h>
extern bool newConfig;
extern uint8_t sentConfigurationMsg[MSG_LEN];
extern uint8_t whichLoopIteration;
extern uint8_t foundDevicesCount;


extern volatile uint8_t start_read_tx_char_handle;
extern volatile uint8_t start_read_rx_char_handle;
uint8_t whichServerConnecting = 1;

/* USER CODE END */

/* Private defines -----------------------------------------------------------*/
/**
 * Define the role here only if it is not already defined in the project options
 * For the CLIENT_ROLE comment the line below 
 * For the SERVER_ROLE uncomment the line below
 */
//#define SERVER_ROLE //Client role.

#define BDADDR_SIZE 6
 
/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;

#ifdef SERVER_ROLE
  BLE_RoleTypeDef BLE_Role = SERVER;
#else
  BLE_RoleTypeDef BLE_Role = CLIENT;
#endif

extern volatile uint8_t set_connectable;
extern volatile uint8_t all_servers_connected;
extern volatile uint8_t notification_enabled;
extern volatile uint8_t client_ready;
extern volatile uint8_t discovery_started;
extern volatile uint8_t discovery_finished;
extern volatile uint16_t connectionHandles[];
extern volatile bool services_discovered;

extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Process(void);
static void User_Init(void);

/* USER CODE BEGIN PFP */
void delayMicrosecondsBLE(uint32_t us);
/* USER CODE END PFP */

#if PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
/**
 * @brief  This function is a utility to print the log time
 *         in the format HH:MM:SS:MSS (DK GUI time format)
 * @param  None
 * @retval None
 */
void print_csv_time(void){
  uint32_t ms = HAL_GetTick();
  PRINT_CSV("%02d:%02d:%02d.%03d", ms/(60*60*1000)%24, ms/(60*1000)%60, (ms/1000)%60, ms%1000);
}
#endif

void MX_BlueNRG_MS_Init(void)
{
  /* USER CODE BEGIN SV */ 
  /* USER CODE END SV */
  
  /* USER CODE BEGIN BlueNRG_MS_Init_PreTreatment */
  /* USER CODE END BlueNRG_MS_Init_PreTreatment */

  /* Initialize the peripherals and the BLE Stack */
  uint8_t CLIENT_BDADDR[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  uint8_t  hwVersion;
  uint16_t fwVersion;
  int ret;
  
  /* Initialize the button, LED, and UART (UART2!) */
  User_Init();
  
  /* Get the User Button initial state */
  user_button_init_state = BSP_PB_GetState(BUTTON_KEY);
  
  /* ! Podczepienie wskaznika na funkcje user_notify (sprawdzajaca typ eventu i wywolujaca odpowiednia funkcje), ...
   * inicjalizacja warstwy transportowej i kolejki pakietow danych */
  hci_init(user_notify, NULL);
      
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  printf("HWver %d, FWver %d\n", hwVersion, fwVersion);

  if (hwVersion > 0x30) { /* Yes, X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1;
  }

  /* 
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  hci_reset();
  
  /* Wpisanie trybu pracy urzadzenia: tryb 3 - "Mode 3: master/slave, 8 connections, RAM1 and RAM2." */
  const uint8_t stackMode[] = {0x03};
  ret = aci_hal_write_config_data(CONFIG_DATA_MODE_OFFSET,
                                  CONFIG_DATA_MODE_LEN,
                                  stackMode);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Setting stack mode failed 0x%02x.\n", ret);
  }

  /* Skopiowanie i wpisanie adresu urzadzenia bdaddr */
  BLUENRG_memcpy(bdaddr, CLIENT_BDADDR, sizeof(CLIENT_BDADDR));
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
								  CONFIG_DATA_PUBADDR_LEN,
								  bdaddr);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Setting BD_ADDR failed 0x%02x.\n", ret);
  }
  
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Setting power level failed 0x%02x.\n", ret);
  }

  /* GATT init? */
  ret = aci_gatt_init();    
  if (ret != BLE_STATUS_SUCCESS) {
    printf("GATT_Init failed.\n");
  }
  
  /* Klient: Inicjalizacja GAP (Generic Access Profile) - ustawia role urzadzenia, sciaga handle do nazwy i (nieuzywany?)do serwisow */
  ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0x00, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if (ret != BLE_STATUS_SUCCESS) {
    printf("GAP_Init failed.\n");
  }


  //? nie ma tego w skryptach
  /* Wymagania autoryzacji - w tym ustalony (staly) kod pin, typ klucza autoryzacji, tryb laczenia */
//  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
//                                     OOB_AUTH_DATA_ABSENT,
//                                     NULL,
//                                     7,
//                                     16,
//                                     USE_FIXED_PIN_FOR_PAIRING,
//                                     123456,
//                                     BONDING);
//  if (ret == BLE_STATUS_SUCCESS) {
//    printf("BLE Stack Initialized.\n");
//  }
  
  printf("CLIENT: BLE Stack Initialized\n");

  /* USER CODE BEGIN BlueNRG_MS_Init_PostTreatment */
  
  /* USER CODE END BlueNRG_MS_Init_PostTreatment */
}

/*
 * BlueNRG-MS background task
 */
void MX_BlueNRG_MS_Process(void)
{
  /* USER CODE BEGIN BlueNRG_MS_Process_PreTreatment */
  
  /* USER CODE END BlueNRG_MS_Process_PreTreatment */
  
  /* Stworzenie (nie nawiazanie) polaczenia (master) lub ustawienie wykrywalnosci (slave) i ?wlaczenie powiadomien, ?udostepnienie charakterystyk */
  /* Po nawiazaniu polaczenia nic juz sie w User_Process nie dzieje! */
  User_Process();
  /* Przeparsuj otrzymane pakiety i wywolaj odpowiednie funkcje; tu sa odebrane dane */
  hci_user_evt_proc();

  /* USER CODE BEGIN BlueNRG_MS_Process_PostTreatment */
  
  /* USER CODE END BlueNRG_MS_Process_PostTreatment */
}

/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  BSP_LED_Init(LED2);
    
  BSP_COM_Init(COM1); 
}

/**
 * @brief  Configure the device as Client or Server and manage the communication
 *         between a client and a server.
 *
 * @param  None
 * @retval None
 */
static void User_Process(void)
{
	/* Wyszukiwanie serverow przez klienta */
	if(!discovery_started){
		tBleStatus ret = aci_gap_start_general_discovery_proc(0x10, 0x10, PUBLIC_ADDR, 0x01); //filter duplicates
		//The two devices are discovered through the EVT_LE_ADVERTISING_REPORT events (sample_service.c).
		if (ret != BLE_STATUS_SUCCESS) {
			printf("Error starting device discovery process!\r\n");
		}
		discovery_started = TRUE;
	}

	if(set_connectable && discovery_finished)
	{
		/* Establish connection with first remote device */
		if(foundDevicesCount > 0){
			Make_Connection(); /* Stworzenie (nie nawiazanie) polaczenia (master) lub ustawienie wykrywalnosci (slave) */
		}
		else{
			printf("No connectable devices found!\r\n");
		}
		set_connectable = FALSE;
	}

	//na razie nie wiem, czy ten sposob do czegos prowadzi -> na razie rezygnuje z niego
	//tBleStatus aci_gatt_disc_all_prim_services(uint16_t conn_handle)
	//This command will start the GATT client procedure to discover all primary services on the server.
//	if(all_servers_connected && !services_discovered){
//		tBleStatus ret = aci_gatt_disc_all_prim_services(connectionHandles[0]);
//		if (ret != BLE_STATUS_SUCCESS) {
//			printf("Error starting service discovery process!\r\n");
//		}
//		//NIE tutaj
//		services_discovered = TRUE;
//	}


	//TODO: sprobowac tego co nizej na stalych i tych samych dla kazdego servera UUID
    /* Start TX handle Characteristic dynamic discovery if not yet done */
	/* z user_notify ustawiamy all_servers_connected po nawiazaniu polaczenia = Skad jest wywolywane GAP_ConnectionComplete_CB */
    if (all_servers_connected && !end_read_tx_char_handle){
      startReadTXCharHandle(); //trzeba wiedziec, od ktorego polaczenia!
    }
//    /* Start RX handle Characteristic dynamic discovery if not yet done */
//    else if (all_servers_connected && !end_read_rx_char_handle){
//      startReadRXCharHandle();
//    }
//
//    if (/*connected &&*/ end_read_tx_char_handle && end_read_rx_char_handle && !notification_enabled)
//    {
//      BSP_LED_Off(LED2); /* end of the connection and chars discovery phase */
//      enableNotification(); /* Wlacz wymiane danych? */
//    }
//
//
//    /* Klient wysyla dane serwerowi TODO: lepiej zeby to bylo w sample_service.c? */
//    //
//    //proba wyslania konfiguracji - dopiero gdy oba servery polaczone, do drugiego servera
////    if(client_ready)
//    if(client_ready && all_servers_connected && end_read_tx_char_handle && end_read_rx_char_handle
//      && notification_enabled && whichServerConnecting == 2)
//    {
//		if(newConfig == true){
//		  newConfig = false; //TODO: problem - wiadomosc z konfiguracja moze byc gubiona, nie sprawdzam tego! rozwiazanie - ACK?
//		  sendData(sentConfigurationMsg, sizeof(sentConfigurationMsg));
//		}
//    	/* Wymiana danych dla konfiguracji zdalnej slave'a:
//    	 * 1. master wysyla do slave'a info o konfiguracji w odpowiednim formacie (np. sekwencja rodzaj_sensora adres_pinu)
//    	 * 2. slave przeparsuje sobie ta konfiguracje i odczyta info jakie ma sensory i na jakich pinach
//    	 * 3. slave wywola funckje do odczytania z tych sensorow ktore dostal w konfiguracji
//    	 * 4. funkcja od odczytywania z wielu sensorow da znac (?) gdy bedzie miec juz wszystkie dane
//    	 * 5. slave wysle odpowiednia liczbe bajtow danych (obliczona wczesniej na podst. konfiguracji) masterowi
//    	 * */
//    }
//
//    if (all_servers_connected && end_read_tx_char_handle && end_read_rx_char_handle && notification_enabled && !client_ready)
//    {
//    	client_ready = true;
//    }
//
//    //
//    if(client_ready && whichServerConnecting == 1){
//		set_connectable = true;
//		//connected = false;
//		start_read_tx_char_handle = false;
//		start_read_rx_char_handle = false;
//		end_read_tx_char_handle = false;
//		end_read_rx_char_handle = false;
//		notification_enabled = false;
//		whichServerConnecting++;
//
//
//		//!
//		client_ready = false;
//
//    }
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Set the User Button flag */
  user_button_pressed = 1;
}

/* USER CODE BEGIN 0 */
void delayMicrosecondsBLE(uint32_t us){
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
/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_X_CUBE_BLE1_C */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
