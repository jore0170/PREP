/***************************************************************************//**
 * @file app.c
 * @brief application code for parallel tx/rx broadcasting to ble characteristic
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
#define BUFFER_LENGTH 512
#define GATTDB_GATT_SPP_DATA 26
/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <stdio.h>
#include <stdlib.h>
#include "app.h"

typedef struct {
	int8  cbuf[BUFFER_LENGTH];
	int  data_length;
	uint32_t size;
	uint32_t read_ptr;
	uint32_t write_ptr;

} BLE_CIRCULAR_BUF;

bd_addr local_addr;
BLE_CIRCULAR_BUF ble_cbuf;
uint16 result3;
uint16 result2;
uint8 _conn_handle;
bool Connection;

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
//result3 = gecko_cmd_gatt_server_send_characteristic_notification(_conn_handle, GATTDB_GATT_SPP_DATA, _max_packet_size, temp)->result;
result3 = gecko_cmd_gatt_write_characteristic_value_without_response(_conn_handle, GATTDB_GATT_SPP_DATA, _max_packet_size, temp)->result;
printLog("Write response %x", result3);
//resetting non connected device makes both devices gather data
//connection handle of 1 because that is what the connection is defined as in advertiser
//GATTDB_GATT_SPP_DATA = 26 because that is what the SPP example code had it macro'd as
ble_cbuf.data_length = 0;
return false;

}

void ble_circ_push(int data){
	//if (ble_cbuf.data_length == 0){
		//for (int i=0; i<=5; i++){
		//ble_cbuf.cbuf[ble_cbuf.write_ptr + i] = local_addr.addr[5-i];
		//update_circ_wrtindex(&ble_cbuf, 1);
		//ble_cbuf.size --;
		//ble_cbuf.data_length ++;
		//}
	//}else{
	ble_cbuf.cbuf[ble_cbuf.write_ptr] = data; //populate the circular buffer with each char of string
	update_circ_wrtindex(&ble_cbuf, 1); // move the write ptr 1
	ble_cbuf.size --;
	ble_cbuf.data_length++;
	//}
}


static void update_circ_wrtindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by){
index_struct->write_ptr = (index_struct->write_ptr + update_by)&(BUFFER_LENGTH-1);//macro for 511
}

static void update_circ_readindex(BLE_CIRCULAR_BUF *index_struct, uint32_t update_by){
index_struct->read_ptr = (index_struct->read_ptr + update_by)&(BUFFER_LENGTH-1);//macro for 511
}

static void reset_variables()
{

	_max_packet_size = 20;

}

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
static uint8_t boot_to_dfu = 0;
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

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Set the maximum number of periodic sync allowed*/
  pconfig->bluetooth.max_periodic_sync = 2;

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();

  /* Initialize stack */
  gecko_init(pconfig);

  /* Initialize Periodic advertisement */
  gecko_init_periodic_advertising();

  //Initialize CTE Transmitter
  gecko_bgapi_class_cte_transmitter_init();

  //Initialize CTE Receiver
  gecko_bgapi_class_cte_receiver_init();

  uint8 sync_handle;
  gecko_bgapi_class_sync_init();

  while (1) {

	  if (ble_cbuf.data_length >= _max_packet_size ){
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
    evt = gecko_wait_event();

    /* Handle events */
	switch (BGLIB_MSG_ID(evt->header)) {
	  /* This boot event is generated when the system boots up after reset.
	   * Do not call any stack commands before receiving the boot event.
	   * Here the system is set to start advertising immediately after boot procedure. */

	/* Array for advertisement data */
	      static uint8 periodic_adv_data[191];
	      uint16 result;

	  case gecko_evt_system_boot_id:
		  reset_variables();
		  gecko_cmd_gatt_set_max_mtu(247);



		  bootMessage(&(evt->data.evt_system_boot));
		  gecko_cmd_system_set_tx_power(0);
		  gecko_cmd_le_gap_set_advertise_tx_power(0,0);// was (0,30)


          gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

          /* turn off legacy PDU flag*/
          gecko_cmd_le_gap_clear_advertise_configuration(0,1);
          gecko_cmd_le_gap_clear_advertise_configuration(0x1,0);//check

          /* It is recommended to enable event le_gap_extended_scan_response
		  which contains useful information for establishing a synchronization. */
		  gecko_cmd_le_gap_set_discovery_extended_scan_response(true);
		  gecko_cmd_le_gap_set_discovery_timing(le_gap_phy_1m,200,200);
		  gecko_cmd_le_gap_set_discovery_type(le_gap_phy_1m,0);
		  gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);

		  // Start extended advertising
		  result = gecko_cmd_le_gap_start_advertising(0,le_gap_general_discoverable, le_gap_non_connectable)->result;
		  printf("le_gap_start_advertising() returns 0x%X\r\n", result);

		  // Start periodic advertising
		  /* adv set #1 , 100 ms min/max interval, include tx power in PDU*/
		  //min and max were 160
		  result = gecko_cmd_le_gap_start_periodic_advertising(0,0x06,0x06,1)->result;
		  printf("start_periodic_advertising returns 0x%X\r\n",result);

		  //advertising for connectable computer
		  result2 = gecko_cmd_le_gap_start_advertising(0x1, le_gap_general_discoverable, le_gap_connectable_scannable)->result;
		  printLog("Connection Advertising Response 0x%x \r\n",result2);

		  //configure & enable cte after periodic advertising enabled

		  uint8 thandle = 0;
		  uint8 cte_length = 0x02;//was decimal 5, hex 2 is minimum (16us)
		  uint8 cte_type = 0;
		  uint8 tcte_count = 1;
		  uint8 s_len = 1;
		  uint8 sa[1] = {0};

		  uint16 response = gecko_cmd_cte_transmitter_enable_connectionless_cte(thandle,cte_length,cte_type,tcte_count,s_len,sa)->result;
		  printf("Response: 0x%x \r\n",response);

		  //Set BT5 advertisement data
		  result = gecko_cmd_le_gap_bt5_set_adv_data(0,8,sizeof(periodic_adv_data),periodic_adv_data)->result;
		  //printf("set_adv_data for periodic advertising data returns 0x%X\r\n",result);
		  gecko_cmd_hardware_set_soft_timer(32768, 0, 0);//handle 0

		break;
	  case gecko_evt_le_connection_opened_id:
		  Connection = true;
	       _conn_handle = evt->data.evt_le_connection_opened.connection;
	       printLog("Connected\r\n");
	      /* Request connection parameter update.
	      * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
	     * supervision timeout 2 seconds
	     * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
	     gecko_cmd_le_connection_set_timing_parameters(0x1, 24, 40, 0, 200, 0, 0xFFFF);
	    break;
	  case gecko_evt_le_connection_closed_id:
		  gecko_cmd_cte_receiver_disable_connection_cte(sync_handle);
		  printf("Connection Closed\r\n");\

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
	  case gecko_evt_hardware_soft_timer_id:
		  if (evt->data.evt_hardware_soft_timer.handle == 0){
	          	for (int i = 0; i < 190; i++) {
	          	    periodic_adv_data[i] = rand()%9;
	          	}

	          	//Reset the advertising data pointers to reflect the change in data
	          	result = gecko_cmd_le_gap_bt5_set_adv_data(0,8,sizeof(periodic_adv_data),periodic_adv_data)->result;
	          	//printf("set_adv_data for periodic advertising data returns 0x%X \r\n",result);
		  }else{
			  gecko_cmd_sync_close(sync_handle);
		  }
	          	break;
	  case gecko_evt_le_gap_extended_scan_response_id:
	  {

		  /* only look at extended advertisements */
		  if(evt->data.evt_le_gap_extended_scan_response.packet_type & 0x80){
			  printLog("got ext adv indication with tx_power = %d\r\n",
					  evt->data.evt_le_gap_extended_scan_response.tx_power );
			  if (findServiceInAdvertisement(&(evt->data.evt_le_gap_extended_scan_response.data.data[0]), evt->data.evt_le_gap_extended_scan_response.data.len) != 0) {

				  printLog("found periodic sync service, attempting to open sync\r\n");

				  uint16 skip = 1, timeout = 40; /* timeout was 20 */
				  sync_handle = gecko_cmd_sync_open(evt->data.evt_le_gap_extended_scan_response.adv_sid,
						 skip,
						 timeout,
						 evt->data.evt_le_gap_extended_scan_response.address,
						 evt->data.evt_le_gap_extended_scan_response.address_type)->sync;
				  printLog("cmd_sync_open() sync = 0x%2X\r\n", sync_handle);


			  }
		  }
	  }
	  break;

	  case gecko_evt_sync_opened_id:
		  /* now that sync is open, we can stop scanning*/
		  printLog("evt_sync_opened\r\n");
		  gecko_cmd_hardware_set_soft_timer(0,1,0);//handle 1
		  gecko_cmd_le_gap_end_procedure();

		  //configure & enable cte after periodic sync established
		  uint8 rhandle = sync_handle;
		  uint8 slot_dur = 1;//switching and sampling are 1us each, other option is 2
		  uint8 rcte_count = 0; //reporting all, other values are max cte's sampled in each periodic advertising interval


		  uint16 res = gecko_cmd_cte_receiver_enable_connectionless_cte(rhandle, slot_dur,rcte_count,s_len,sa)->result;

		  printf("Response: 0x%x\r\n",res);
		  break;

	  case gecko_evt_sync_closed_id:
		  printLog("periodic sync closed. reason 0x%2X, sync handle %d\r\n",
				  evt->data.evt_sync_closed.reason,
				  evt->data.evt_sync_closed.sync);
		  /* restart discovery */
		  gecko_cmd_le_gap_start_discovery(le_gap_phy_1m,le_gap_discover_observation);
		  break;

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
		    printf("GOT CONNECTIONLESS IQ report\r\n");
		    struct gecko_msg_cte_receiver_connectionless_iq_report_evt_t report = evt->data.evt_cte_receiver_connectionless_iq_report;
			//printf("GOT SILABS IQ report\r\n");
			//printf("status: %d, ch: %d, rssi: %d, ant:%d, cte:%d, duration:%d, len:%d\r\n", report.status,
					//report.channel, report.rssi, report.rssi_antenna_id, report.cte_type, report.slot_durations,
					//report.samples.len);
			//for (int i=0; i<report.samples.len; i++) {
				//RETARGET_WriteChar(report.samples.data[i]);
			//}
		    if (Connection){
			ble_circ_push(report.channel);
			ble_circ_push(report.rssi);
		    }
		    //printf("\r\n");
		  break;
	  }
	  default:
		break;
	}
  }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
#if DEBUG_LEVEL
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
