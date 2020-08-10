/**
  ******************************************************************************
  * @file    sample_service.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Add a sample service using a vendor specific profile.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "sample_service.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include <stdbool.h>

/** @addtogroup Applications
 *  @{
 */

/** @addtogroup SampleAppThT
 *  @{
 */
 
/** @defgroup SAMPLE_SERVICE 
 * @{
 */
#define RSSI_DETAILS_SIZE 8

/** @defgroup SAMPLE_SERVICE_Private_Variables
 * @{
 */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t connectedDevicesCount = 0;
volatile uint8_t pairedDevicesCount = 0;
volatile bool set_connectable = true;
volatile bool client_ready = false;
volatile bool discovery_started = false;
volatile bool discovery_finished = false;
volatile bool start_notifications_enable = false;
volatile bool all_notifications_enabled = false;
volatile bool start_read_tx_char_handle = false;//?
volatile bool start_read_rx_char_handle = false;//?
volatile bool all_tx_char_handles_read = false;
volatile bool all_rx_char_handles_read = false;
volatile bool all_servers_connected = false;
volatile bool pairing_started = false;
volatile bool pairing_finished = false;
volatile bool services_discovered = false;
/* Klient zna handle do charakterystyk serverow */
uint16_t txHandles[MAX_CONNECTIONS];
uint8_t txHandlesDiscoveredCount = 0;
uint16_t rxHandles[MAX_CONNECTIONS];
uint8_t rxHandlesDiscoveredCount = 0;
uint8_t notifications_enabled_count = 0;
/* Server zna handle do swojego serwisu i swoich charakterystyk */
uint16_t sampleServHandle, TXCharHandle, RXCharHandle;

//typedef-y do struktur -> w .h
//FoundDeviceInfo foundDevices[MAX_CONNECTIONS]; //main.cpp
uint8_t foundDevicesCount = 0;

/**
 * @}
 */
 
/** @defgroup SAMPLE_SERVICE_Private_Macros
 * @{
 */
/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
  	uuid_struct.uuid128[0] = uuid_0; uuid_struct.uuid128[1] = uuid_1; uuid_struct.uuid128[2] = uuid_2; uuid_struct.uuid128[3] = uuid_3; \
	uuid_struct.uuid128[4] = uuid_4; uuid_struct.uuid128[5] = uuid_5; uuid_struct.uuid128[6] = uuid_6; uuid_struct.uuid128[7] = uuid_7; \
	uuid_struct.uuid128[8] = uuid_8; uuid_struct.uuid128[9] = uuid_9; uuid_struct.uuid128[10] = uuid_10; uuid_struct.uuid128[11] = uuid_11; \
	uuid_struct.uuid128[12] = uuid_12; uuid_struct.uuid128[13] = uuid_13; uuid_struct.uuid128[14] = uuid_14; uuid_struct.uuid128[15] = uuid_15; \
	}while(0)
/**
 * @}
 */

/** @defgroup SAMPLE_SERVICE_Exported_Functions 
 * @{
 */ 
/**
 * @brief  Add a sample service using a vendor specific profile
 * @param  None
 * @retval Status
 */
tBleStatus Add_Sample_Service(void)
{
  tBleStatus ret;
  
  /*
  UUIDs:
  D973F2E0-B19E-11E2-9E96-0800200C9A66
  D973F2E1-B19E-11E2-9E96-0800200C9A66
  D973F2E2-B19E-11E2-9E96-0800200C9A66
  */
  
  const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
  const uint8_t charUuidTX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9}; //roznica na bajcie 12
  const uint8_t charUuidRX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9}; //roznica na bajcie 12
  
  /* Dodanie jednego glownego serwisu o service_uuid jak wyzej i zapisanie handle'a do tego serwisu w sampleServHandle */
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &sampleServHandle); /* original is 9?? */
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  /* Dodanie dwoch charakterystyk: TX i RX do stworzonego glownego serwisu, zapisanie handle'i do tych serwisow w TXCharHandle i RXCharH. */
  ret =  aci_gatt_add_char(sampleServHandle, UUID_TYPE_128, charUuidTX, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 1, &TXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  ret =  aci_gatt_add_char(sampleServHandle, UUID_TYPE_128, charUuidRX, 20, CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &RXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  PRINTF("Sample Service added.\nTX Char Handle %04X, RX Char Handle %04X\n", TXCharHandle, RXCharHandle);
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding Sample Service.\n");
  return BLE_STATUS_ERROR ;
}

/**
 * @brief  Make the device connectable
 * @param  None 
 * @retval None
 */
void Make_Connection(void)
{  
	printf("Client Create Connection with device %d\r\n\r\n", connectedDevicesCount+1);
	tBleStatus ret = aci_gap_create_connection(
		SCAN_P/*0x0010*/, /* 10240 msec = Scan Interval: from when the Controller started its last scan until it begins the subsequent scan = how long to wait between scans (for a number N, Time = N x 0.625 msec) */
		SCAN_L/*0x0010*/, /* 10240 msec = Scan Window: amount of time for the duration of the LE scan = how long to scan (for a number N, Time = N x 0.625 msec) */
		foundDevices[connectedDevicesCount].deviceAddressType, /* Peer_Address_Type */
		foundDevices[connectedDevicesCount].deviceAddress, /* Peer_Address */
		PUBLIC_ADDR, /* Own_Address_Type */
		CONN_P1/*0x06C*/, /* 50 msec = Minimum Connection Period (interval) = time between two data transfer events (for a number N, Time = N x 1.25 msec) */
		CONN_P2/*0x06C*/, /* 50 msec = Maximum Connection Period (interval) = Connection interval is the time between one radio event on a given connection and the next radio event on the same connection (for a number N, Time = N x 1.25 msec) */
		0x0000, /* Connection latency = If non-zero, the peripheral is allowed to skip up to slave latency radio events and not listen. That saves even more power, at the expense of even slower data. number of consecutive connection events where the slave doesn't need to listen on the master(?) */
		SUPERV_TIMEOUT/*0x0C80*/, /* 600 msec = Supervision Timeout (reset upon reception of a valid packet) max time between 2 packets before connection is considered lost (Time = N x 10 msec) */
		/*CONN_L1*/0x000C, /* 7.5 msec (previously 1250 msec and not working) = Minimum Connection Length (for a number N, Time = N x 0.625 msec) */
		/*CONN_L2*/0x000C  /* 7.5 msec (previously 1250 msec and not working) = Maximal Connection Length (for a number N, Time = N x 0.625 msec) */
	);
	if (ret != BLE_STATUS_SUCCESS){
	  printf("Error while starting connection with device %d\r\n\r\n", connectedDevicesCount);
	}
}

/**
 * @brief Start the pairing process with a scanned device
 * @param none
 * @retval none
 */
void Pair_Devices(void){
	tBleStatus ret = aci_gap_send_pairing_request(/*conn_handle*/foundDevices[pairedDevicesCount].connHandle, /*force_rebond*/0x01);
	//0x00: Pairing request is sent only if the device has not previously bonded
	//0x01: Pairing request will be sent even if the device was previously bonded
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error starting device pairing process with device %d\r\n\r\n", pairedDevicesCount+1);
	}
	else{
		printf("Started pairing with device %d\r\n\r\n", pairedDevicesCount+1);
	}
}

/**
 * @brief  Discovery TX characteristic handle by UUID 128 bits
 * @param  None 
 * @retval None
 */
void startReadTXCharHandle(void)
{
  if (!all_tx_char_handles_read)
  {    
    PRINTF("Start reading TX Char Handle\n");
    start_read_tx_char_handle = TRUE;
	const uint8_t charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(foundDevices[txHandlesDiscoveredCount].connHandle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_TX);
  }
}

/**
 * @brief  Discovery RX characteristic handle by UUID 128 bits
 * @param  None 
 * @retval None
 */
void startReadRXCharHandle(void)
{  
  if (!all_rx_char_handles_read)
  {
    PRINTF("Start reading RX Char Handle\n");
    start_read_rx_char_handle = TRUE;
    const uint8_t charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(foundDevices[rxHandlesDiscoveredCount].connHandle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_RX);
  }
}

/**
 * @brief  This function is used to receive data related to the sample service
 *         (received over the air from the remote board).
 * @param  data_buffer : pointer to store in received data
 * @param  Nb_bytes : number of bytes to be received
 * @retval None
 */
void receiveData(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  BSP_LED_Off(LED2);
  for(int i = 0; i < Nb_bytes; i++) {
	  printf("%c", data_buffer[i]);
  }
  fflush(stdout);
}

/**
 * @brief  This function is used to send data related to the sample service
 *         (to be sent over the air to the remote board).
 * @param  server_index : index of the server which will receive data
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
//void sendData(uint8_t* data_buffer, uint8_t Nb_bytes)
void sendData(uint8_t server_index, uint8_t* data_buffer, uint8_t Nb_bytes)
{
	/* Klient wysyla dane do serwera */
	//TODO wysylanie do konkretnego serwera (adres lub nazwa)
	uint8_t server_ind = (server_index <= connectedDevicesCount ? server_index : connectedDevicesCount); /* Aby wyslac do polaczonego servera */
    aci_gatt_write_without_response(foundDevices[server_ind].connHandle, rxHandles[server_ind]+1, Nb_bytes, data_buffer); /* Max 20 bajtow na jedno wywolanie funkcji, serwer nie potwierdza otrzymania pakietu */
	//aci_gatt_write_charac_value(connection_handle, rx_handle+1, Nb_bytes, data_buffer); /* The client provides a handle and the contents of the value (up to ATT_MTU-3 bytes, because the handle and the ATT operation code are included in the packet with the data) and the server will !acknowledge the write operation with a response! */
	//aci_gatt_write_long_charac_val(connection_handle, rx_handle+1, 0, Nb_bytes, data_buffer); /* This permits a client to write more than ATT_MTU-3 bytes of data into a server’s characteristic value or descriptor. It works by queueing several prepare write operations, each of which includes an offset and the data itself, and then finally writing them all atomically with an execute write operation. */
}

/**
 * @brief  Enable notification
 * @param  None 
 * @retval None
 */
void enableNotification(void)
{
  uint8_t client_char_conf_data[] = {0x01, 0x00}; /* Enable notifications */
  uint32_t tickstart = HAL_GetTick();
  start_notifications_enable = TRUE;
  
  while(notifications_enabled_count < connectedDevicesCount){
	  while(aci_gatt_write_charac_descriptor(foundDevices[notifications_enabled_count].connHandle, txHandles[notifications_enabled_count]+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){ /* ? */
		/* Radio is busy */
		if ((HAL_GetTick() - tickstart) > (10*HCI_DEFAULT_TIMEOUT_MS)) break;
	  }
	  foundDevices[notifications_enabled_count].connStatus = READY_TO_EXCHANGE_DATA;
	  notifications_enabled_count++;
  }
  all_notifications_enabled = TRUE; //?
  printf("All notifications enabled!\r\nReady to exchange data!\r\n\r\n");
}

/**
 * @brief  This function is called when an attribute gets modified
 * @param  handle : handle of the attribute
 * @param  data_length : size of the modified attribute data
 * @param  att_data : pointer to the modified attribute data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if(handle == RXCharHandle + 1){
    receiveData(att_data, data_length);
  } else if (handle == TXCharHandle + 2) {        
    if(att_data[0] == 0x01)
    	start_notifications_enable = TRUE; //?
  }
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  addr : Address of peer device
 * @param  handle : Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  foundDevices[connectedDevicesCount].connHandle = handle;
  foundDevices[connectedDevicesCount].connStatus = CONNECTED;
  connectedDevicesCount++;
  printf("Connected to device: ");
  for(int i = 5; i > 0; i--){
    printf("%02X-", addr[i]);
  }
  printf("%02X\r\n\r\n", addr[0]);
  printf("Connection handle: %04X\r\n\r\n", handle);
  if(connectedDevicesCount < foundDevicesCount){
	  Make_Connection();
  }
  else if(connectedDevicesCount == foundDevicesCount){
	  all_servers_connected = TRUE;
	  printf("All found devices connected!\r\n\r\n");
  }
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  all_servers_connected = FALSE;
  
  printf("Disconnected\n");

  //TODO: przemyslec zachowanie klienta po rozlaczeniu polaczenia!
  //connectedDevicesCount--; //trzeba jeszcze odnalezc, ktore urz. odlaczone
  /* Make the device connectable again. */
//  set_connectable = TRUE;
//  start_notifications_enable = FALSE;
//  start_read_tx_char_handle = FALSE;
//  start_read_rx_char_handle = FALSE;
//  all_tx_char_handles_read = FALSE;
//  all_rx_char_handles_read = FALSE;
}

/**
 * @brief  This function is called when there is a notification from the sever.
 * @param  attr_handle Handle of the attribute
 * @param  attr_len    Length of attribute value in the notification
 * @param  attr_value  Attribute value in the notification
 * @retval None
 */
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value, uint16_t conn_handle)
{
	/* !Odebrane dane od servera! */
    if (attr_handle == txHandles[txHandlesDiscoveredCount-1]+1 && attr_len != 0 && *attr_value != '\0') { //TODO check txHandles[txHandlesDiscoveredCount] -> wszystkie handle sa takie same, rozrozniac urzadzenia mozna po pozycji w connectionHandles[] i po adresie z foundDevices[]
    	xSemaphoreTake(newDataMutexHandle, DELAY_TIME);
    	memset((char *)dataBLE[newData], '\0', MSG_LEN);
    	for(int i=0; i<=attr_len && i<=MSG_LEN; i++){ //nazwa czujnika, dane
    		dataBLE[newData][i] = *(attr_value+i);
    	}
    	//handle do polaczenia
    	dataBLE[newData][attr_len] = ((conn_handle >> 8) & 0xFF);
    	dataBLE[newData][attr_len+1] = (conn_handle & 0xFF);

      	if(newData+1<MAX_MSGS){
      		newData++;
      	}
      	xSemaphoreGive(newDataMutexHandle);
    }
}

/**
 * @brief  This function is called when the client found an advertising device to which it may want to connect.
 * @param  adv_info		Structure containing information about the advertising device
 * @retval None
 */
void GAP_AdvertisingReport_CB(le_advertising_info *adv_info){
	if(adv_info->evt_type == ADV_IND){
		//sprawdzic, czy znalezione urzadzenie to nie duplikat (czy nie mamy juz tego adresu w tablicy)
		bool isDeviceNew = true;
		tBDAddr foundDeviceAddress;
		memset(foundDeviceAddress, 0, BD_ADDR_SIZE);
		memcpy(foundDeviceAddress, adv_info->bdaddr, BD_ADDR_SIZE);
		for(int i=0; i<foundDevicesCount; i++){
			if(memcmp(foundDeviceAddress, foundDevices[i].deviceAddress, BD_ADDR_SIZE) == 0){
				isDeviceNew = false;
			}
		}
		if(isDeviceNew){
			printf("New device found!\r\n\r\n");
			//zapisac nazwe urzadzenia
			uint8_t name_len = adv_info->data_length-BD_ADDR_SIZE-RSSI_DETAILS_SIZE; //wybrac tylko bajty dot. nazwy
			uint8_t device_name[name_len];
			memset(device_name, 0, name_len);
			memcpy(device_name, adv_info->data_RSSI+5, name_len); //nazwa urzadzenia przesunieta o 5 bajtow w data_RSSI[]
			//dodac nowo znalezione urzadzenie do tablicy pamietanych urzadzen, zwiekszyc liczbe pamietanych urzadzen
			foundDevices[foundDevicesCount].deviceAddressType = adv_info->bdaddr_type;
			memcpy(foundDevices[foundDevicesCount].deviceAddress, adv_info->bdaddr, BD_ADDR_SIZE);
			//nazwa i status
			if(name_len > MAX_NAME_LEN) { name_len = MAX_NAME_LEN; }
			memcpy(foundDevices[foundDevicesCount].deviceName, adv_info->data_RSSI+5, name_len);
			foundDevices[foundDevicesCount].connStatus = READY_TO_CONNECT;
			if(foundDevicesCount+1<MAX_CONNECTIONS){ foundDevicesCount++; }
		}
	} /* evt_type == ADV_IND */
	else if(adv_info->evt_type == SCAN_RSP){
		uint8_t pin_len = adv_info->data_length;
		uint8_t received_pin[pin_len];
		memset(received_pin, 0, pin_len);
		memcpy(received_pin, adv_info->data_RSSI, pin_len);
		const uint8_t correct_pin[6] = {'8','3','1','6','2','9'};
		if(memcmp(received_pin, correct_pin, pin_len) != 0){
			//pin nie zgadza sie = nie chcemy sie laczyc z tym urzadzeniem = usuwamy je z foundDevices[] i czyscimy jego dane
			printf("Wrong authentication code from device %d - unable to connect\r\n\r\n", foundDevicesCount);
			foundDevices[foundDevicesCount].deviceAddressType = 0;
			memset(foundDevices[foundDevicesCount].deviceAddress, 0, BD_ADDR_SIZE);
			memset(foundDevices[foundDevicesCount].deviceName, 0, MAX_NAME_LEN);
			foundDevices[foundDevicesCount].connStatus = DISCONNECTED;
			if(foundDevicesCount > 0){ foundDevicesCount--; } //ew. sprawdzic czy nie usuwamy ostatniego urzadzenia
		}
	}
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  pData  Pointer to the ACI packet
 * @retval None
 */
void user_notify(void * pData) /* Parsowanie otrzymanego eventu */
{
  hci_uart_pckt *hci_pckt = pData;  
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
    return;
  
  switch(event_pckt->evt){

	  /* Disconnection */
	  case EVT_DISCONN_COMPLETE:
		{
		  //TODO: po rozlaczeniu: odpowiedni komunikat, zmiana statusu odp. polaczenia na DISCCONNECTED i ew. ponowne wyszukiwanie
		  GAP_DisconnectionComplete_CB();
		}
		break;

	  /* Connection Complete, Advertising Report */
	  case EVT_LE_META_EVENT:
		{
		  evt_le_meta_event *evt = (void *)event_pckt->data;

		  switch(evt->subevent){
			  case EVT_LE_CONN_COMPLETE:
				{
				  evt_le_connection_complete *cc = (void *)evt->data;
				  GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
				}
				break;

				/* Klient znalazl urzadzenie podczas skanowania */
				case EVT_LE_ADVERTISING_REPORT:
				{
				  //wyciagnac info o typie adresu, adres i nazwe urzadzenia
				  le_advertising_info *adv_info = (void*)(evt->data+1); /*evt->data[0] is number of reports (On BlueNRG-MS is always 1)*/
				  GAP_AdvertisingReport_CB(adv_info);
				}
				break;
		  }
		}
		break;

	  /* rozne typy eventow EVT_VENDOR: */
	  case EVT_VENDOR:
		{
		  evt_blue_aci *blue_evt = (void*)event_pckt->data;
		  switch(blue_evt->ecode){

			  /* Attribute modified (zmieniaja sie handle TX i RX) - server odbiera dane */
			  case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
				{
				  if (bnrg_expansion_board == IDB05A1) {
					evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
					Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
				  }
				  else {
					evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
					Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
				  }

				}
				break;

			  /* GATT notification = odebrane dane od servera */
			  case EVT_BLUE_GATT_NOTIFICATION:
				{
				  evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
				  /* mozna jeszcze wykorzystac handle do polaczenia, ktory przychodzi razem z pakietem */
				  GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value, evt->conn_handle);
				  /* a w callbacku odbior otrzymanych danych do jakiejs tablicy */
				}
				break;

			  /* Odczytwanie charakterystyk slave'a czyli zapamietywanie TX i RX handles */
			  case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
			  {
				if(BLE_Role == CLIENT) {
				  PRINTF("EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP\n");

				  evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;

				  if (start_read_tx_char_handle && !all_tx_char_handles_read)
				  {
					txHandles[txHandlesDiscoveredCount] = resp->attr_handle;
					//printf("TX Char Handle %04X\r\n\r\n", txHandles[txHandlesDiscoveredCount]);
					//dalej (sprawdzenie czy to juz wszystkie ch-tyki TX) juz w EVT_BLUE_GATT_PROCEDURE_COMPLETE
				  }
				  if (start_read_rx_char_handle && !all_rx_char_handles_read)
				  {
					rxHandles[rxHandlesDiscoveredCount] = resp->attr_handle;
					//printf("RX Char Handle %04X\r\n\r\n", rxHandles[rxHandlesDiscoveredCount]);
					//dalej (sprawdzenie czy to juz wszystkie ch-tyki RX) juz w EVT_BLUE_GATT_PROCEDURE_COMPLETE
				  }
				}
			  }
				break;

			  /* Potrzebne dla mastera w UserProcess */
			  case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
			    if(BLE_Role == CLIENT) {
				  evt_gatt_procedure_complete *evt = (void *)blue_evt->data;
				  //chyba tylko przez wartosci zmiennych globalnych da sie ustalic co sie stalo, tak jak nizej
				  if(evt->error_code == BLE_STATUS_SUCCESS){
					  //printf("EVT_BLUE_GATT_PROCEDURE_COMPLETE\r\n"); //jaka dokladnie procedura? trzeba sprawdzac po zmiennych glob.!
					  if (start_read_tx_char_handle && !all_tx_char_handles_read)
					  {
						txHandlesDiscoveredCount++;
						if(txHandlesDiscoveredCount == connectedDevicesCount){
							all_tx_char_handles_read = TRUE;
							printf("All TX handles read!\r\n\r\n");
						}
						else if(txHandlesDiscoveredCount < connectedDevicesCount){
							startReadTXCharHandle();
						}
					  }
					  if (start_read_rx_char_handle && !all_rx_char_handles_read)
					  {
						rxHandlesDiscoveredCount++;
						if(rxHandlesDiscoveredCount == connectedDevicesCount){
							all_rx_char_handles_read = TRUE;
							printf("All RX handles read!\r\n\r\n");
						}
						else if(rxHandlesDiscoveredCount < connectedDevicesCount){
							startReadRXCharHandle();
						}
					  }
				  }
				}
				break;

			  /* Koniec skanowania klienta w poszukiwaniu serverow */
			  case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
			  {
				  evt_gap_procedure_complete *evt = (evt_gap_procedure_complete *)blue_evt->data;
				  if(evt->procedure_code == GAP_GENERAL_DISCOVERY_PROC && evt->status == BLE_STATUS_SUCCESS){
					  printf("Scanning for devices finished\r\n\r\n");
					  discovery_finished = TRUE;
				  }
			  }
			  break;

			  /* Koniec procesu parowania urzadzen */
			  case EVT_BLUE_GAP_PAIRING_CMPLT:
			  {
				  evt_gap_pairing_cmplt *evt = (evt_gap_pairing_cmplt *)blue_evt->data;
				  if(evt->status == 0x00){ //pairing success
					  foundDevices[pairedDevicesCount].connStatus = PAIRED;
					  printf("Paired with device %d\r\n\r\n", ++pairedDevicesCount);
					  if(pairedDevicesCount < foundDevicesCount){
						  Pair_Devices();
					  }
					  else if(pairedDevicesCount == foundDevicesCount){
						  pairing_finished = true;
						  printf("Paired with all found devices\r\n\r\n");
					  }
				  }
			  }
			  break;
		  }
		} /* EVT_VENDOR */
		break;
  }    
}
/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
