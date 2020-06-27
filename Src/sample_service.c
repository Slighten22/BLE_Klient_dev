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

#include "app_x-cube-ble1.h"
#include <stdbool.h>

extern uint8_t dataBLE[];
extern bool newData;
extern uint8_t whichLoopIteration;

/** @addtogroup Applications
 *  @{
 */

/** @addtogroup SampleAppThT
 *  @{
 */
 
/** @defgroup SAMPLE_SERVICE 
 * @{
 */

/** @defgroup SAMPLE_SERVICE_Private_Variables
 * @{
 */
/* Private variables ---------------------------------------------------------*/
volatile int connected = FALSE;
volatile uint8_t set_connectable = 1;
volatile uint16_t connection_handle = 0;
volatile uint8_t notification_enabled = FALSE;
volatile uint8_t start_read_tx_char_handle = FALSE;
volatile uint8_t start_read_rx_char_handle = FALSE;
volatile uint8_t end_read_tx_char_handle = FALSE;
volatile uint8_t end_read_rx_char_handle = FALSE;

volatile uint8_t client_ready = FALSE;

uint16_t tx_handle; /* Klient zna handle do charakterystyk servera */
uint16_t rx_handle;

uint16_t sampleServHandle, TXCharHandle, RXCharHandle; /* Server zna handle do swojego serwisu i swoich charakterystyk */

extern uint8_t bnrg_expansion_board;
extern BLE_RoleTypeDef BLE_Role;
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
  tBleStatus ret;
  
  if(BLE_Role == CLIENT) {
    
    printf("Client Create Connection\n");
    

    //TODO nowy adres servera do polaczenia
//    tBDAddr bdaddr = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02}; /* Adres servera */
    tBDAddr bdaddr = {0xcc, 0x00, 0x00, 0xE1, 0x80, 0x02}; /* Adres servera */


    BSP_LED_On(LED2); //To indicate the start of the connection and discovery phase
    
    /*
    Scan_Interval, Scan_Window, Peer_Address_Type, Peer_Address, Own_Address_Type, Conn_Interval_Min, 
    Conn_Interval_Max, Conn_Latency, Supervision_Timeout, Conn_Len_Min, Conn_Len_Max    
    */
    ret = aci_gap_create_connection(
    		SCAN_P, /* 10240 msec = Time interval from when the Controller started its last scan until it begins the subsequent scan = how long to wait between scans (for a number N, Time = N x 0.625 msec) */
    		SCAN_L, /* 10240 msec = Scan Window: amount of time for the duration of the LE scan = how long to scan (for a number N, Time = N x 0.625 msec) */
			PUBLIC_ADDR, /* Peer_Address_Type */
			bdaddr, /* Peer_Address */
			PUBLIC_ADDR, /* Own_Address_Type */
			CONN_P1, /* 50 msec = Minimum Connection Period (interval) = time between two data transfer events (for a number N, Time = N x 1.25 msec) */
			CONN_P2, /* 50 msec = Maximum Connection Period (interval) = Connection interval is the time between one radio event on a given connection and the next radio event on the same connection (for a number N, Time = N x 1.25 msec) */
			0, /* Connection latency = If non-zero, the peripheral is allowed to skip up to slave latency radio events and not listen. That saves even more power, at the expense of even slower data. number of consecutive connection events where the slave doesn't need to listen on the master(?) */
            SUPERV_TIMEOUT, /* 600 msec = Supervision Timeout (reset upon reception of a valid packet) max time between 2 packets before connection is considered lost (Time = N x 10 msec) */
			CONN_L1, /* 1250 msec = Minimum Connection Length (for a number N, Time = N x 0.625 msec) */
			CONN_L2  /* 1250 msec = Maximal Connection Length (for a number N, Time = N x 0.625 msec) */
	);
    
    if (ret != 0){
      printf("Error while starting connection.\n");
    }
    
  } else  {
    
    const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G','_','C','h','a','t'};
    
    /* disable scan response (nie bedize dodatkowych informacji o serverze dla mastera) */
    hci_le_set_scan_resp_data(0,NULL);
    
    PRINTF("General Discoverable Mode ");
    /*
    Advertising_Event_Type, Adv_Interval_Min, Adv_Interval_Max, Address_Type, Adv_Filter_Policy,
    Local_Name_Length, Local_Name, Service_Uuid_Length, Service_Uuid_List, Slave_Conn_Interval_Min,
    Slave_Conn_Interval_Max
    */
    ret = aci_gap_set_discoverable(
    		ADV_DATA_TYPE, /* undirected scannable and connectable = Advertising_Event_Type */
			ADV_INTERV_MIN,/* 1280 msec = Minimum Advertising Interval https://www.argenox.com/library/bluetooth-low-energy/ble-advertising-primer/(for a number N, Time = N x 0.625 msec) */
			ADV_INTERV_MAX,/* 2560 msec = Maximal Advertising Interval (slow advertising, non-critical latency) (for a number N, Time = N x 0.625 msec) */
			PUBLIC_ADDR,   /* public address = Address_Type */
            NO_WHITE_LIST_USE,/* Process scan and connection requests from all devices (i.e., the White List is not in use) */
			13, /* Local_Name_Length (= 0x09BlueNRG_Chat) */
			local_name, /* 0x09BlueNRG_Chat */
			0, /* Service_Uuid_Length */
			NULL, /* Service_Uuid_List */
			0, /* Slave_Conn_Interval_Min = time between one radio event on a given connection and the next radio event on the same connection */
			0  /* Slave_Conn_Interval_Max https://devzone.nordicsemi.com/f/nordic-q-a/25340/do-i-understand-ble-connection-interval-properly */
	);
    PRINTF("%d\n",ret);
  }
}

/**
 * @brief  Discovery TX characteristic handle by UUID 128 bits
 * @param  None 
 * @retval None
 */
void startReadTXCharHandle(void)
{
  if (!start_read_tx_char_handle)
  {    
    PRINTF("Start reading TX Char Handle\n");
    
    //TODO inne charakterystyki TX, RX (roznica na 3. bajcie)
    const uint8_t charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,/*0xe1*/0xe4,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_TX);
    start_read_tx_char_handle = TRUE;
  }
}

/**
 * @brief  Discovery RX characteristic handle by UUID 128 bits
 * @param  None 
 * @retval None
 */
void startReadRXCharHandle(void)
{  
  if (!start_read_rx_char_handle)
  {
    PRINTF("Start reading RX Char Handle\n");
    
    //TODO inne charakterystyki TX, RX (roznica na 3. bajcie)
    const uint8_t charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,/*0xe2*/0xe5,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_RX);
    start_read_rx_char_handle = TRUE;
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
  } //obudowac do wysylania
  // tu moze byc obudowa uartowa/samo wysylanie wartosci temp i wilgotnosci
  fflush(stdout);
}

/**
 * @brief  This function is used to send data related to the sample service
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
void sendData(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  if(BLE_Role == SERVER) {    
    aci_gatt_update_char_value(sampleServHandle,TXCharHandle, 0, Nb_bytes, data_buffer);    
  }
  else { /* Klient do serwera */
    aci_gatt_write_without_response(connection_handle, rx_handle+1, Nb_bytes, data_buffer); /* Max 20 bajtow na jedno wywolanie funkcji, serwer nie potwierdza otrzymania pakietu */
	//aci_gatt_write_charac_value(connection_handle, rx_handle+1, Nb_bytes, data_buffer); /* The client provides a handle and the contents of the value (up to ATT_MTU-3 bytes, because the handle and the ATT operation code are included in the packet with the data) and the server will !acknowledge the write operation with a response! */
	//aci_gatt_write_long_charac_val(connection_handle, rx_handle+1, 0, Nb_bytes, data_buffer); /* This permits a client to write more than ATT_MTU-3 bytes of data into a server’s characteristic value or descriptor. It works by queueing several prepare write operations, each of which includes an offset and the data itself, and then finally writing them all atomically with an execute write operation. */
  }
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
  
  while(aci_gatt_write_charac_descriptor(connection_handle, tx_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){ /* ? */
    /* Radio is busy */
    if ((HAL_GetTick() - tickstart) > (10*HCI_DEFAULT_TIMEOUT_MS)) break;
  }
  notification_enabled = TRUE;
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
      notification_enabled = TRUE;
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
  connected = TRUE;
  connection_handle = handle;
  
  printf("Connected to device:");
  for(int i = 5; i > 0; i--){
    printf("%02X-", addr[i]);
  }
  printf("%02X\n", addr[0]);
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  
  printf("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
  start_read_tx_char_handle = FALSE;
  start_read_rx_char_handle = FALSE;
  end_read_tx_char_handle = FALSE;
  end_read_rx_char_handle = FALSE;
}

/**
 * @brief  This function is called when there is a notification from the sever.
 * @param  attr_handle Handle of the attribute
 * @param  attr_len    Length of attribute value in the notification
 * @param  attr_value  Attribute value in the notification
 * @retval None
 */
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{
	/* Odebrane dane od servera */
    if (attr_handle == tx_handle+1 && attr_len != 0 && *attr_value != '\0') {
	  for(int i=0; i<attr_len && i<20; i++){
		  dataBLE[i] = *(attr_value+i);
	  }
	  if(client_ready){
		  newData = true;
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
      GAP_DisconnectionComplete_CB();
    }
    break;
    
  /* Connection Complete */
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
      }
    }
    break;
    
  /* 4 rozne typy eventow EVT_VENDOR: */
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
        
      /* Attribute modified (zmieniaja sie handle TX i RX) */
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
          GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value);
          /* a w callbacku odbior otrzymanych danych do jakiejs tablicy */
        }
        break;

      /* Odczytwanie charakterystyk slave'a czyli poznawanie TX i RX handles */
      case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
        if(BLE_Role == CLIENT) {
          PRINTF("EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP\n");
          
          evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;
          
          if (start_read_tx_char_handle && !end_read_tx_char_handle)
          {
            tx_handle = resp->attr_handle;
            printf("TX Char Handle %04X\n", tx_handle);
          }
          else if (start_read_rx_char_handle && !end_read_rx_char_handle)
          {
            rx_handle = resp->attr_handle;
            printf("RX Char Handle %04X\n", rx_handle);
          }
        }
        break;
        
      /* Potrzebne dla mastera w UserProcess */
      case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
        if(BLE_Role == CLIENT) {
          /* Wait for gatt procedure complete event trigger related to Discovery Charac by UUID */
          //evt_gatt_procedure_complete *pr = (void*)blue_evt->data;
          
          if (start_read_tx_char_handle && !end_read_tx_char_handle)
          {
            end_read_tx_char_handle = TRUE;
          }
          else if (start_read_rx_char_handle && !end_read_rx_char_handle)
          {
            end_read_rx_char_handle = TRUE;
          }
        }
        break;
      }
    }
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
