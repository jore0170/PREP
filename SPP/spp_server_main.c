/***********************************************************************************************//**
 * \file   spp_server_main.c
 * \brief  SPP server example
 *
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"


#include <stdio.h>
#include "retargetserial.h"
#include "sleep.h"
#include "spp_utils.h"
#include "em_usart.h"

/***************************************************************************************************
  Local Macros and Definitions
 **************************************************************************************************/

#define STATE_ADVERTISING 1
#define STATE_CONNECTED   2
#define STATE_SPP_MODE    3
#define VALUE_LEN         1
/* Maximum number of iterations when polling UART RX data before sending data over BLE connection
 * set value to 0 to disable optimization -> minimum latency but may decrease throughput */
#define UART_POLL_TIMEOUT  5000
#define SERVICE_UUID_LENGTH 16
#define CHARACTERISTIC_UUID_LENGTH 16
//4880c12c-fdcb-4077-8920-a450d7f9b907

#define RESPONSE_LEN_BYTES 2
#define ONE_S (32768U)
#define TEST_DURATION_TIMER (1)
#define TX
#define PHY_NUMBER     (5) /* actually there only 4 options but the enumeration starts at 1 */
#define BUFFER_LENGTH   64
/***************************************************************************************************
 Local Variables
 **************************************************************************************************/
static uint8 _conn_handle = 0xFF;
static int _main_state;


uint8 value_data[VALUE_LEN] = {7};
struct gecko_msg_gatt_characteristic_evt_t* characteristic;
struct gecko_msg_gatt_service_evt_t* service;


tsCounters _sCounters;

static uint8 _max_packet_size = 20; // Maximum bytes per one packet
static uint8 _min_packet_size = 20; // Target minimum bytes for one packet

uint8 service_uuid_length = 16;
const uint8 service_uuid[SERVICE_UUID_LENGTH] = {0x07, 0xb9, 0xf9, 0xd7, 0x50, 0xa4, 0x20, 0x89, 0x77, 0x40, 0xcb, 0xfd, 0x2c, 0xc1, 0x80, 0x48};
uint8 characteristic_uuid_length = 16;
const uint8 characteristic_uuid[CHARACTERISTIC_UUID_LENGTH] = {0xd6, 0x58, 0xd6, 0x21, 0xbc, 0x55, 0x81, 0x9f, 0x42, 0x44, 0x71, 0x6d, 0xc4, 0x6e, 0xc2, 0xfe};

struct gecko_msg_gatt_server_characteristic_status_evt_t *pStatus;

gecko_configuration_t *pconfig;
struct dtm_data{
    // eDTM_TYPE eType; /* Best would be to use a status word/register type of thing */
    // union dmt_data_status status;

    uint8 result[RESPONSE_LEN_BYTES];
    uint32 test_duration;
    struct gecko_msg_test_dtm_completed_evt_t event;
    struct gecko_msg_test_dtm_rx_cmd_t rx_cmd;
    struct gecko_msg_test_dtm_rx_rsp_t rx_rsp;
    struct gecko_msg_test_dtm_tx_cmd_t tx_cmd;
    struct gecko_msg_test_dtm_tx_rsp_t tx_rsp;
};

typedef struct {
	char  cbuf[512];
	uint8_t  size_mask;
	uint32_t size;
	uint32_t read_ptr;
	uint32_t write_ptr;

} BLE_CIRCULAR_BUF;

BLE_CIRCULAR_BUF ble_cbuf;

uint16 *result2;
//static const char* phy_desc[PHY_NUMBER] = { "invalid",
//                                            "1M",
//                                            "2M",
//                                            "125K",
//                                            "500k" };
//static struct dtm_data dtm_test_data;
static void update_circ_wrtindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by);
static void update_circ_readindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by);
static uint8_t ble_circ_space(void);


uint8 testlength = 8;
uint16 offset = 0;
uint8_t test = 7;
uint8 *rssi;
uint16 *mtu;
void ble_circ_push(char *string){
//imlement the loop here with the header, and payload
	uint8_t space = ble_circ_space();//update the availability of the buffer
	int string_length = strlen(string); //find the length of the string
	char header = strlen(string)+1; //add a byte for the header's value


	if(space > string_length){
		ble_cbuf.cbuf[ble_cbuf.write_ptr] = header; //set the header to the total packet length
		update_circ_wrtindex(&ble_cbuf, 1); //move the write ptr 1

		for (int i = 0; i < string_length; i++){
			ble_cbuf.cbuf[ble_cbuf.write_ptr] = string[i]; //populate the circular buffer with each char of string
			update_circ_wrtindex(&ble_cbuf, 1); // move the write ptr 1
		}
	}else if((_max_packet_size - (BUFFER_LENGTH - space)) < 10){
		ble_circ_pop();
	}else{
		EFM_ASSERT(false);
	}
	ble_cbuf.size -= 1+ string_length;
}

bool ble_circ_pop(){
if (ble_circ_space()==BUFFER_LENGTH){ //buffer is empty
		return true;
}
char print_str[BUFFER_LENGTH];
int string_length = (int)(ble_cbuf.cbuf[ble_cbuf.read_ptr]) - 1;
update_circ_readindex(&ble_cbuf, 1); //move the read ptr 1
ble_cbuf.size += 1;//see if this fixes //increase availability of buffer
	for(int i = 0; i < string_length; i++){
		print_str[i] = ble_cbuf.cbuf[ble_cbuf.read_ptr]; //move the packet to a print variable for transmission
		update_circ_readindex(&ble_cbuf, 1); //update
		ble_cbuf.size += 1; //add a byte back to the size
	}
gecko_cmd_gatt_write_characteristic_value_without_response(_conn_handle, pStatus->characteristic, 1, rssi);
return false;

}

static uint8_t ble_circ_space(void){
return ble_cbuf.size;
}

static void update_circ_wrtindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by){
index_struct->write_ptr = (index_struct->write_ptr + update_by)&(BUFFER_LENGTH-1);//macro for 511
}

static void update_circ_readindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by){
index_struct->read_ptr = (index_struct->read_ptr + update_by)&(BUFFER_LENGTH-1);//macro for 511
}

static void reset_variables()
{
	_conn_handle = 0xFF;
	_main_state = STATE_ADVERTISING;

	_max_packet_size = 20;

	memset(&_sCounters, 0, sizeof(_sCounters));
}




static void send_spp_data()
{
	uint8 len = 0;
	uint8 data[256];
	uint16 result;

	int c;
	int timeout = 0;

	// Read up to _max_packet_size characters from local buffer
	while (len < _max_packet_size) {
		  c = RETARGET_ReadChar();

		  if(c >= 0) {
			  data[len++] = (uint8)c;
		  } else if(len == 0) {
			  /* If the first ReadChar() fails then return immediately */
			  return;
		  } else {
			  /* Speed optimization: if there are some bytes to be sent but the length is still
			   * below the preferred minimum packet size, then wait for additional bytes
			   * until timeout. Target is to put as many bytes as possible into each air packet */

			  // Conditions for exiting the while loop and proceed to send data:
			  if(timeout++ > UART_POLL_TIMEOUT) {
				  break;
			  } else if(len >= _min_packet_size) {
				  break;
			  }
		  }
	}

	if (len > 0) {
		// Stack may return "out-of-memory" error if the local buffer is full -> in that case, just keep trying until the command succeeds
		do {
			result = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, gattdb_gatt_spp_data, len, data)->result;
			_sCounters.num_writes++;
		} while(result == bg_err_out_of_memory);

		if (result != 0) {
			printLog("Unexpected error: %x\r\n", result);
		} else {
			_sCounters.num_pack_sent++;
			_sCounters.num_bytes_sent += len;
		}
	}
}

/**
 * @brief  SPP server mode main loop
 */
void spp_server_main(void)
{
	int i;
	//pconfig->bluetooth.max_connections = 3;
	//check for running in parallel
  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;
    
    if(_main_state == STATE_SPP_MODE) {
    	/* If SPP data mode is active, use non-blocking gecko_peek_event() */
    	evt = gecko_peek_event();

    	if(evt == NULL) {
    		/* No stack events to be handled -> send data from local TX buffer */
    		send_spp_data();
    		continue;  		// Jump directly to next iteration i.e. call gecko_peek_event() again
    	}
    } else {
    	/* SPP data mode not active -> check for stack events using the blocking API */
    	evt = gecko_wait_event();
    }

    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {

      /* This boot event is generated when the system boots up after reset.
       * Here the system is set to start advertising immediately after boot procedure. */
    case gecko_evt_system_boot_id:

    	reset_variables();
    	gecko_cmd_gatt_set_max_mtu(247);

    	gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_undirected_connectable);
    	break;


    	/* Connection opened event */
    case gecko_evt_le_connection_opened_id:

    	_conn_handle = evt->data.evt_le_connection_opened.connection;
    	printLog("Connected\r\n");
    	_main_state = STATE_CONNECTED;

    	/* Request connection parameter update.
    	 * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
    	 * supervision timeout 2 seconds
    	 * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
    	gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);

    	//setting timer for periodic writes
    	gecko_cmd_hardware_set_soft_timer(163840,2,0); //5 seconds, id = 2, 0 =repeating

    	//unhardcode later
    	struct gecko_msg_gatt_discover_primary_services_by_uuid_rsp_t *result;
    	result = gecko_cmd_gatt_discover_primary_services_by_uuid(_conn_handle, service_uuid_length, service_uuid);
    	printLog("First \n");
    	printLog("result %d \n", result->result);
    	printLog("Second \n");
    	//this should take you to gatt_service event
    	break;

    case gecko_evt_le_connection_parameters_id:
    	printLog("Conn.parameters: interval %u units, txsize %u\r\n", evt->data.evt_le_connection_parameters.interval, evt->data.evt_le_connection_parameters.txsize);

    	break;

    case gecko_evt_gatt_mtu_exchanged_id:
    	/* Calculate maximum data per one notification / write-without-response, this depends on the MTU.
    	 * up to ATT_MTU-3 bytes can be sent at once  */
    	_max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
    	_min_packet_size = _max_packet_size; /* Try to send maximum length packets whenever possible */
    	printLog("MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
    	break;
    //case gecko_rsp_gatt_server_get_mtu_id:
    	//mtu = evt->data.rsp_gatt_server_get_mtu.mtu;
    //break;
    case gecko_evt_le_connection_closed_id:
    	printLog("DISCONNECTED!\r\n");

    	/* Show statistics (RX/TX counters) after disconnect: */
    	printStats(&_sCounters);

    	reset_variables();
    	SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping

    	/* Restart advertising */
    	gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_undirected_connectable);
    	break;

    case gecko_evt_gatt_server_characteristic_status_id:
    {
    	//struct gecko_msg_gatt_server_characteristic_status_evt_t *pStatus;
    	pStatus = &(evt->data.evt_gatt_server_characteristic_status);

    	if (pStatus->characteristic == gattdb_gatt_spp_data) {
    		if (pStatus->status_flags == gatt_server_client_config) {
    			// Characteristic client configuration (CCC) for spp_data has been changed
    			if (pStatus->client_config_flags == gatt_notification) {
    				_main_state = STATE_SPP_MODE;
    				SLEEP_SleepBlockBegin(sleepEM2); // Disable sleeping
    				printLog("SPP Mode ON\r\n");
    			} else {
    				printLog("SPP Mode OFF\r\n");
    				_main_state = STATE_CONNECTED;
    				SLEEP_SleepBlockEnd(sleepEM2); // Enable sleeping
    			}

    		}
    	}
    }
    break;

    case gecko_evt_gatt_server_attribute_value_id:
    {
    	 for(i=0;i<evt->data.evt_gatt_server_attribute_value.value.len;i++) {
    		 USART_Tx(RETARGET_UART, evt->data.evt_gatt_server_attribute_value.value.data[i]);
    	 }

    	 _sCounters.num_pack_received++;
    	 _sCounters.num_bytes_received += evt->data.evt_gatt_server_attribute_value.value.len;
    }
    break;
    case gecko_evt_hardware_soft_timer_id:
    	printLog("\n soft timer \n");
    	gecko_cmd_le_connection_get_rssi(_conn_handle);
    break;
    case gecko_evt_le_connection_rssi_id:

    	*rssi = evt->data.evt_le_connection_rssi.rssi;
    	ble_circ_push(rssi);
    	gecko_cmd_gatt_write_characteristic_value_without_response(_conn_handle, pStatus->characteristic, 1, rssi);
    	*result2 = evt->data.rsp_gatt_write_characteristic_value_without_response.result;
    	printLog("Written");
    	printLog("write response %p", result2);

    break;
    default:
    	break;
    }
  }
}
