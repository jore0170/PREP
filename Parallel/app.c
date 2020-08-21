/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Periodic Advertisement Scanner Program
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "app.h"
#include "retargetserial.h"

/* Timer and GPIO libraries*/
#include "em_timer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_msc.h"
#include "em_prs.h"

#define RX_OBS_PRS_CHANNEL 0
#define BUFFER_LENGTH 512


/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;

struct Advertising_Parameters {
	bd_addr     local_addr;
	bd_addr	    connection_addr;
	uint8_t     ad_handle;
	uint16_t 	min_interval;
	uint16_t 	max_interval;
	uint8_t		ch_map[5];
};

struct CTE_TX_params {
	uint8_t		cte_length;
	uint8_t		cte_type;
	uint8_t		cte_count;
	uint8_t		s_len;
	uint8_t		sa[1];
};
struct CTE_RX_params {
	uint8_t		sync_handle;
	uint8_t		slot_dur;
	uint8_t		cte_count;
	uint8_t		s_len;
	uint8_t		sa[1];
};

static struct Advertising_Parameters ad_params;
static struct CTE_RX_params RX_params;
static struct CTE_TX_params TX_params;

typedef struct {
	int8  cbuf[BUFFER_LENGTH];
	int  data_length;
	uint32_t size;
	uint32_t read_ptr;
	uint32_t write_ptr;

} BLE_CIRCULAR_BUF;

BLE_CIRCULAR_BUF ble_cbuf;
uint16 result3;
uint16 result2;
uint8 _conn_handle;
bool Connection;
uint8 channel;
int8 rssi;


static uint8 _max_packet_size = 20; // Maximum bytes per one packet
static uint8 _min_packet_size = 20; // Target minimum bytes for one packet
static void update_circ_wrtindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by);
static void update_circ_readindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by);

bool ble_circ_pop(){
uint8 temp[_max_packet_size];
	for(int i = 0; i < ble_cbuf.data_length; i++){
		temp[i] = ble_cbuf.cbuf[ble_cbuf.read_ptr]; //move the packet to a print variable for transmission
		update_circ_readindex(&ble_cbuf, 1); //update
		ble_cbuf.size += 1; //add a byte back to the size
	}

uint16 result4 = gecko_cmd_gatt_server_send_characteristic_notification(0xff, gattdb_Data, _max_packet_size, temp)->result;
printLog("Notification Response %x",result4);
//1st param was _conn_handle
/*try conn handle of 0xff to send no matter what*/
//GATTDB_GATT_SPP_DATA = 26 because thats what it enums to after generating with gatt.xml
ble_cbuf.data_length = 0;
gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);
return false;

}

void ble_circ_push(int data){
	ble_cbuf.cbuf[ble_cbuf.write_ptr] = data; //populate the circular buffer with each char of string
	update_circ_wrtindex(&ble_cbuf, 1); // move the write ptr 1
	ble_cbuf.size --;
	ble_cbuf.data_length++;
}


static void update_circ_wrtindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by){
index_struct->write_ptr = (index_struct->write_ptr + update_by)&(BUFFER_LENGTH-1);//macro for 511
}

static void update_circ_readindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by){
index_struct->read_ptr = (index_struct->read_ptr + update_by)&(BUFFER_LENGTH-1);//macro for 511
}

bool compare_bd_addr(bd_addr *address1, bd_addr *address2){
	for(int i = 0;i<5;i++){
		if(address1->addr[5-i] > address2->addr[i]){
			return false;
		}
	}
	return true;
}
//
const uint8 periodicSyncService[16] = {0x81,0xc2,0x00,0x2d,0x31,0xf4,0xb0,0xbf,0x2b,0x42,0x49,0x68,0xc7,0x25,0x71,0x41};

// Parse advertisements looking for advertised periodicSync Service.
static uint8_t findServiceInAdvertisement(uint8_t *data, uint8_t len)
{
  uint8_t adFieldLength;
  uint8_t adFieldType;
  uint8_t i = 0;
  printLog("packet length %d\r\n", len);
  // Parse advertisement packet
  while (i < len) {
    adFieldLength = data[i];
    adFieldType = data[i + 1];
    // Partial ($02) or complete ($03) list of 128-bit UUIDs
	printLog("adField type %d \r\n", adFieldType);
    if (adFieldType == 0x06 || adFieldType == 0x07) {
      // compare UUID to service UUID
      if (memcmp(&data[i + 2], periodicSyncService, 16) == 0) {
        return 1;
      }
    }
    // advance to the next AD struct
    i = i + adFieldLength + 1;
  }
  return 0;
}

static volatile uint32_t overflow = 5;
//static volatile uint32_t* ad_data_ptr[4];
uint8_t temp[16];


/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

	/* Set the maximum number of periodic sync allowed*/
	pconfig->bluetooth.max_periodic_sync = 1;

	/* Set maximum number of periodic advertisers */
	pconfig->bluetooth.max_advertisers = 2;

	/* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
	initLog();

	/* Initialize stack */
	gecko_init(pconfig);

	gecko_init_periodic_advertising();

	uint8 sync_handle;
	gecko_bgapi_class_sync_init();

	// Initialize advertising parameters
	ad_params.min_interval 	= 120;
	ad_params.max_interval 	= 120;
	ad_params.ad_handle 	= 0;
	ad_params.ch_map[0]   	= 3;
	for(int i = 1; i<5; i++)
		ad_params.ch_map[i] = 0;

	// Initialize CTE TX parameters
	TX_params.cte_length 	= 0x14;
	TX_params.cte_type		= 0x0;
	TX_params.cte_count 	= 1;
	TX_params.s_len 		= 1;
	TX_params.sa[0] 		= 0;

	// Initialize CTE RX parameters
	RX_params.slot_dur		= 1;
	RX_params.cte_count 	= 0;
	RX_params.s_len 		= 1;
	RX_params.sa[0] 		= 0;


	//Initialize CTE Reciever & Transmitter
	//gecko_bgapi_class_cte_receiver_init();
	//gecko_bgapi_class_cte_transmitter_init();

	uint16 result;
	//uint32_t offset;
	uint32_t* data = malloc(16);
	uint8_t itr = 0;
	//uint8_t* ram = malloc(184*10);

	while (1) {
		if (ble_cbuf.data_length >= _max_packet_size ){
					  gecko_cmd_le_gap_end_procedure();
			  		  ble_circ_pop();
			  	  }
		/* Event pointer for handling events */
		struct gecko_cmd_packet* evt;

		/* if there are no events pending then the next call to gecko_wait_event() may cause
		* device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
		if (!gecko_event_pending()) {
		flushLog();
		}

		/* Check for stack event. This is a blocking event listener. If you want non-blocking please see UG136. */

		result = gecko_cmd_le_gap_bt5_set_adv_data(0, 8, sizeof(uint32_t)*4, temp)->result;
		evt = gecko_wait_event();

		/* Handle events */
		switch (BGLIB_MSG_ID(evt->header)) {
			case gecko_evt_system_boot_id:

				gecko_cmd_gatt_set_max_mtu(247);

				bootMessage(&(evt->data.evt_system_boot));
				printLog("Periodic Sync scanner system boot\r\n");

				gecko_cmd_system_set_tx_power(100);
				gecko_cmd_le_gap_set_advertise_tx_power(0,30);
				gecko_cmd_le_gap_set_advertise_timing(0, ad_params.max_interval, 160, 0, 0);
				gecko_cmd_le_gap_clear_advertise_configuration(0,1);
				result = gecko_cmd_le_gap_set_data_channel_classification(5, ad_params.ch_map)->result;
				printLog("set data channel classification: %d\r\n", result);

				result = gecko_cmd_le_gap_start_advertising(0,le_gap_general_discoverable, le_gap_non_connectable)->result;
				printLog("le_gap_start_advertising() returns 0x%X\r\n", result);

				/* adv set #1 , 100 ms min/max interval, include tx power in PDU*/
				//TIMER_Enable(TIMER1,true);
				result = gecko_cmd_le_gap_start_periodic_advertising(0,ad_params.min_interval,ad_params.max_interval,1)->result;
				printLog("start_periodic_advertising returns 0x%X\r\n",result);

				//Set BT5 advertisement data

				//result = gecko_cmd_le_gap_bt5_set_adv_data(0,8,sizeof(ad_data),ad_data)->result;
				//printLog("set_adv_data for periodic advertising data returns 0x%X\r\n",result);

				/* It is recommended to enable event le_gap_extended_scan_response
				which contains useful information for establishing a synchronization. */
				gecko_cmd_le_gap_set_discovery_extended_scan_response(true);
				gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m,200,200);
				gecko_cmd_le_gap_set_discovery_type(le_gap_phy_1m,0);
				//gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);
				////////////////////////////////////////////////////////////////////////////////
				result = gecko_cmd_le_gap_start_advertising(1, le_gap_general_discoverable, le_gap_undirected_connectable)->result;
				printLog("Connection Advertising Response =%x \r\n",result);
			break;
			case gecko_evt_le_connection_opened_id:
					  Connection = true;
				       _conn_handle = evt->data.evt_le_connection_opened.connection;
				       printLog("Connected\r\n");
				      /* Request connection parameter update.
				      * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
				     * supervision timeout 2 seconds
				     * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
				     gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);
					gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);

				     //gecko_cmd_hardware_set_soft_timer(32768,2,0);


				    break;

			case gecko_evt_le_connection_closed_id:
				/* Check if need to boot to dfu mode */
				if (boot_to_dfu) {
				/* Enter to DFU OTA mode */
				gecko_cmd_system_reset(2);
				} else {
				/* Restart advertising after client has disconnected */
				gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
				}
			break;
			case gecko_evt_gatt_mtu_exchanged_id:
				      	/* Calculate maximum data per one notification / write-without-response, this depends on the MTU.
				      	 * up to ATT_MTU-3 bytes can be sent at once  */
				      	_max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
				      	_min_packet_size = _max_packet_size; /* Try to send maximum length packets whenever possible */
				      	printLog("MTU exchanged: %d\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
				  break;
			case gecko_evt_le_gap_extended_scan_response_id: {
				if(compare_bd_addr(&ad_params.local_addr, &evt->data.evt_le_gap_extended_scan_response.address)){
					break;
				}
				/* only look at extended advertisements */
				if(evt->data.evt_le_gap_extended_scan_response.packet_type & 0x80){
					printLog("got ext adv indication with tx_power = %d\r\n",
					  evt->data.evt_le_gap_extended_scan_response.tx_power );
					if (findServiceInAdvertisement(&(evt->data.evt_le_gap_extended_scan_response.data.data[0]), evt->data.evt_le_gap_extended_scan_response.data.len) != 0) {

						printLog("found periodic sync service, attempting to open sync\r\n");

						//uint16 skip = 1, timeout = 20;
						//sync_handle = gecko_cmd_sync_open(evt->data.evt_le_gap_extended_scan_response.adv_sid,
							 //skip,
							 //timeout,
							 //evt->data.evt_le_gap_extended_scan_response.address,
							 //evt->data.evt_le_gap_extended_scan_response.address_type)->sync;
						printLog("cmd_sync_open() sync = 0x%2X\r\n", sync_handle);
						//printLog("Channel %d", evt->data.evt_le_gap_extended_scan_response.channel);
						//printLog("RSSI %d", evt->data.evt_le_gap_extended_scan_response.rssi);
						if (Connection){
							ble_circ_push(evt->data.evt_le_gap_extended_scan_response.channel);
							ble_circ_push(evt->data.evt_le_gap_extended_scan_response.rssi);
						}
					}
				}
			}
			break;

			case gecko_evt_sync_opened_id:
				/* now that sync is open, we can stop scanning*/
				printLog("evt_sync_opened\r\n");
				gecko_cmd_le_gap_end_procedure();
				result = gecko_cmd_cte_receiver_enable_connectionless_cte(sync_handle,
								RX_params.slot_dur, RX_params.cte_count, RX_params.s_len, RX_params.sa)->result;
				printLog("Result of gecko_cmd_cte_receiver_enable_connectionless_cte: 0x%x\r\n",result);
				TIMER_Enable(TIMER0,true);
			break;

			case gecko_evt_sync_closed_id:
				printLog("periodic sync closed. reason 0x%2X, sync handle %d",
				evt->data.evt_sync_closed.reason,
				evt->data.evt_sync_closed.sync);
				gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);
			break;

			case gecko_evt_sync_data_id:
				printLog("PERIODIC PACKET RECIEVED\r\n");
				printLog("%d\r\n", evt->data.evt_sync_data.data.len);
				printLog("DATA: %lu %lu %lu %lu \r\n", data[0], data[1], data[2], data[3]);
				data = (uint32_t*) &(evt->data.evt_sync_data.data.data);
				//memcpy(num, data+2,sizeof(uint32_t));
				//(*num)++;
				//memcpy(other_time, data, sizeof(uint32_t));
				//memcpy(other_overflow, data+3, sizeof(uint32_t));
				TIMER_CounterSet(TIMER0, 0);
				overflow = 0;
			break;
			/* Events related to OTA upgrading
			----------------------------------------------------------------------------- */

			/* Check if the user-type OTA Control Characteristic was written.
			* If ota_control was written, boot the device into Device Firmware Upgrade (DFU) mode. */
			case gecko_evt_gatt_server_user_write_request_id:

				if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
					/* Set flag to enter to OTA mode */
					boot_to_dfu = 1;
					/* Send response to Write Request */
					gecko_cmd_gatt_server_send_user_write_response(
					evt->data.evt_gatt_server_user_write_request.connection,
					gattdb_ota_control,
					bg_err_success);

					/* Close connection to enter to DFU OTA mode */
					gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
				}
			break;

			case gecko_evt_cte_receiver_connectionless_iq_report_id: {
				printLog("CTE REPORT\r\n");

				struct gecko_msg_cte_receiver_connectionless_iq_report_evt_t *report =
				&(evt->data.evt_cte_receiver_connectionless_iq_report);

				printLog(
				"status: %d, ch: %d, rssi: %d, ant:%d, cte:%d, duration:%d, sync:%d, event: %d, len:%d\r\n",
				report->status, report->channel, report->rssi, report->rssi_antenna_id,
				report->cte_type, report->slot_durations, report->sync, report->event_counter,
				report->samples.len);

				/*
				for(int i=0; i<4;i++)
					RETARGET_WriteChar(0xFF);
				RETARGET_WriteChar(report->channel);
				RETARGET_WriteChar(report->samples.len);
				RETARGET_WriteChar(num);
				uint8_t* temp = (uint8_t*)&report->event_counter;
				RETARGET_WriteChar(temp[0]);
				RETARGET_WriteChar(temp[1]);
				for (int i=0; i<report->samples.len; i++) {
					RETARGET_WriteChar(report->samples.data[i]);
				}
				RETARGET_WriteChar('\r');
				RETARGET_WriteChar('\n');
				*/

				//uint8_t* userDataPage = ram;
				//offset = (16+report->samples.len);
				if(itr < 9){
					//updateTime(&delta, &old, ad_data);
					//uint8_t* temp = (uint8_t*)&report->event_counter;
					//uint8_t* temp2 = (uint8_t*)other_time;
					//uint8_t* temp3 = (uint8_t*)num;
					//uint8_t* temp4 = (uint8_t*)other_overflow;
					//uint8_t preData[] = {
						//report->channel,
						//report->samples.len,
						//temp[0],
						//temp[1],
						//temp2[0],
						//temp2[1],
						//temp2[2],
						//temp2[3],
						//temp3[0],
						//temp3[1],
						//temp3[2],
						//temp3[3],
						//temp4[0],
						//temp4[1],
						//temp4[2],
						//temp4[3]

					};
					//memcpy(userDataPage+offset*itr, preData, sizeof(preData));
					//memcpy(userDataPage+offset*itr+sizeof(preData), &report->samples.data, report->samples.len);

				}
				itr++;

				if( itr == 8 ){
					uint8_t* ch = malloc(1);
					uint8_t* len = malloc(1);
					uint8_t* event_num = malloc(2);
					uint8_t* num = malloc(4);
					uint8_t* time = malloc(4);
					uint8_t* overflow_n = malloc(4);
					//uint8_t* data = malloc(report->samples.len);
					for(int k=0; k<itr; k++){
						//setTemp();
						//result = gecko_cmd_le_gap_bt5_set_adv_data(0, 8, sizeof(uint32_t)*4, temp)->result;
						for(int i=0; i< 4; i++){
							RETARGET_WriteChar(0xFF);

						//memcpy(ch, userDataPage + 0*sizeof(uint8_t) + offset*k,sizeof(uint8_t));
						RETARGET_WriteChar(*ch);

						//memcpy(len, userDataPage + 1*sizeof(uint8_t) + offset*k,sizeof(uint8_t));
						RETARGET_WriteChar(*len);

						//memcpy(event_num, userDataPage + 2*sizeof(uint8_t) + offset*k,sizeof(uint16_t));
						RETARGET_WriteChar(event_num[0]);
						RETARGET_WriteChar(event_num[1]);

						//memcpy(num, userDataPage + 4*sizeof(uint8_t) + offset*k,sizeof(uint32_t));
						for(int i=0; i<4; i++)
							RETARGET_WriteChar(num[i]);

						//memcpy(time, userDataPage + 8*sizeof(uint8_t) + offset*k,sizeof(uint32_t));
						for(int i=0; i<4; i++)
							RETARGET_WriteChar(time[i]);

						//memcpy(overflow_n, userDataPage + 12*sizeof(uint8_t) + offset*k,sizeof(uint32_t));
						for(int i=0; i<4; i++)
							RETARGET_WriteChar(overflow_n[i]);


						//memcpy(data, userDataPage + 16*sizeof(uint8_t) + offset*k, report->samples.len);
						//for (int i=0; i<report->samples.len; i++) {
							//RETARGET_WriteChar(data[i]);
						//}

						RETARGET_WriteChar('\r');
						RETARGET_WriteChar('\n');
					}
					free(ch);
					free(len);
					free(data);
					free(event_num);
					free(overflow_n);
					free(time);
					itr = 0;
				}

			}
			break;

			default:
			break;
		}
	}
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
	ad_params.local_addr = gecko_cmd_system_get_bt_address()->address;
#if DEBUG_LEVEL
  bd_addr local_addr;
  int i;

  printLog("stack version: %u.%u.%u\r\n", bootevt->major, bootevt->minor, bootevt->patch);
  local_addr = gecko_cmd_system_get_bt_address()->address;

  printLog("local BT device address: ");
  for (i = 0; i < 5; i++) {
    printLog("%2.2x:", local_addr.addr[5 - i]);
  }
  printLog("%2.2x\r\n", local_addr.addr[0]);
#endif
}
