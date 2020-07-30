/***************************************************************************//**
 * @file app.c
 * @brief Silicon Labs Periodic Advertisement Program
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
//ADVERTISER
/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "app.h"

/* Print boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt);

/* Flag for indicating DFU Reset must be performed */
uint8_t boot_to_dfu = 0;
static uint8 _conn_handle = 0xFF;
static uint8 _max_packet_size = 20; // Maximum bytes per one packet
static uint8 _min_packet_size = 20; // Target minimum bytes for one packet
uint16 result2;

static void reset_variables()
{
	_conn_handle = 0xFF;

	_max_packet_size = 20;

}

/* Main application */
void appMain(gecko_configuration_t *pconfig)
{
#if DISABLE_SLEEP > 0
  pconfig->sleep.flags = 0;
#endif

  /* Set maximum number of periodic advertisers */
  pconfig->bluetooth.max_advertisers = 2;
  pconfig->bluetooth.max_connections = 2;

  /* Initialize debug prints. Note: debug prints are off by default. See DEBUG_LEVEL in app.h */
  initLog();


  /* Initialize stack */
  gecko_init(pconfig);

  /* Initialize Periodic advertisement */
  gecko_init_periodic_advertising();

   //Initialize CTE
   gecko_bgapi_class_cte_transmitter_init();

  while (1) {
      /* Event pointer for handling events */
      struct gecko_cmd_packet* evt;

      /* if there are no events pending then the next call to gecko_wait_event() may cause
           * device go to deep sleep. Make sure that debug prints are flushed before going to sleep */
	  if (!gecko_event_pending()) {
		flushLog();
	  }

      /* Check for stack event. */
      evt = gecko_wait_event();

      /* Handle events */
      switch (BGLIB_MSG_ID(evt->header)) {

      /* Array for advertisement data */
      static uint8 periodic_adv_data[191];
      uint16 result;

        /* This boot event is generated when the system boots up after reset.
         * Do not call any stack commands before receiving the boot event.
         * Here the system is set to start advertising immediately after boot procedure. */
        case gecko_evt_system_boot_id:
        reset_variables();
    	gecko_cmd_gatt_set_max_mtu(247);

    	//result2 = gecko_cmd_le_gap_start_advertising(1, le_gap_general_discoverable, le_gap_undirected_connectable)->result;
    	//printLog("Connection Advertising Response =%d \r\n",result2);
        bootMessage(&(evt->data.evt_system_boot));


      	gecko_cmd_system_set_tx_power(0);
      	gecko_cmd_le_gap_set_advertise_tx_power(0,0);// was (0,30)
          /* Set advertising parameters. 100ms advertisement interval.
           * The first parameter is advertising set handle
           * The next two parameters are minimum and maximum advertising interval, both in
           * units of (milliseconds * 1.6).
           * The last two parameters are duration and maxevents left as default. */
      	   //0x20 = decimal 32 would be 20ms (min), 65535 decimal would be 40.96 seconds, 160 decimal is 100ms
          gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);

          /* turn off legacy PDU flag*/
          gecko_cmd_le_gap_clear_advertise_configuration(0,1);

   		  // Start extended advertising
          result = gecko_cmd_le_gap_start_advertising(0,le_gap_general_discoverable, le_gap_non_connectable)->result;
          printf("le_gap_start_advertising() returns 0x%X\r\n", result);

          // Start periodic advertising
          /* adv set #1 , 100 ms min/max interval, include tx power in PDU*/
          //min and max were 160
          result = gecko_cmd_le_gap_start_periodic_advertising(0,0x06,0x06,1)->result;
          printf("start_periodic_advertising returns 0x%X\r\n",result);

          //configure & enable cte after periodic advertising enabled

		   uint8 handle = 0;
		   uint8 cte_length = 0x02;//was decimal 5, hex 2 is minimum (16us)
		   uint8 cte_type = 0;
		   uint8 cte_count = 1;
		   uint8 s_len = 1;
		   uint8 sa[1] = {0};

		   uint16 response = gecko_cmd_cte_transmitter_enable_connectionless_cte(handle,cte_length,cte_type,cte_count,s_len,sa)->result;
		   printf("Response: 0x%x \r\n",response);


          //Set BT5 advertisement data
          result = gecko_cmd_le_gap_bt5_set_adv_data(0,8,sizeof(periodic_adv_data),periodic_adv_data)->result;
          //printf("set_adv_data for periodic advertising data returns 0x%X\r\n",result);
          gecko_cmd_hardware_set_soft_timer(32768, 0, 0);

          break;
        case gecko_evt_le_connection_opened_id:

           	_conn_handle = evt->data.evt_le_connection_opened.connection;
           	printLog("Connected\r\n");
           	//_main_state = STATE_CONNECTED;

           	/* Request connection parameter update.
           	 * conn.interval min 20ms, max 40ms, slave latency 4 intervals,
           	 * supervision timeout 2 seconds
           	 * (These should be compliant with Apple Bluetooth Accessory Design Guidelines, both R7 and R8) */
           	gecko_cmd_le_connection_set_timing_parameters(_conn_handle, 24, 40, 0, 200, 0, 0xFFFF);

           	//setting timer for periodic writes
           	//gecko_cmd_hardware_set_soft_timer((16384/2),2,0); //5 seconds = 163480, id = 2, 0 =repeating
           	//16384 = 0.5s
           	//gecko_cmd_le_gap_start_advertising();
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

        case gecko_evt_hardware_soft_timer_id:
        	//periodic_adv_data = gecko_cmd_system_get_random_data(16)->data;
        	for (int i = 0; i < 190; i++) {
        	    periodic_adv_data[i] = rand()%9;
        	}
        	/*
        	printf("\r\n");
        	for (int i = 0; i < 190; i++) {
        	        	   printf(" %X", periodic_adv_data[i]);
        	        	}
        	printf("\r\n");
        	*/

        	//Reset the advertising data pointers to reflect the change in data
        	result = gecko_cmd_le_gap_bt5_set_adv_data(1,8,sizeof(periodic_adv_data),periodic_adv_data)->result;
        	//printf("set_adv_data for periodic advertising data returns 0x%X \r\n",result);
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

        default:
          break;
      }
    }
}

/* Print stack version and local Bluetooth address as boot message */
static void bootMessage(struct gecko_msg_system_boot_evt_t *bootevt)
{
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



