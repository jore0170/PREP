/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//try copying jrey9/ncs/v1.3 into C:\PREP\ncs
#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#define LED0_NODE DT_ALIAS(led0)
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#if DT_PHA_HAS_CELL(LED0_NODE, gpios, flags)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#endif
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#endif

#ifndef FLAGS
#define FLAGS	0
#endif

//static uint8_t mfg_data[] = { 0xff, 0xff, 0x00 };

/*static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 3),
};*/

const struct bt_le_per_adv_sync_param sync_param = {
  .addr = , //fbab83efeece -> value seems to change each power cycle
  .sid = NULL,
  .options = BT_LE_PER_ADV_SYNC_OPT_NONE,
  .skip = 0x01,
  .timeout =  0x0, 
};

/*The periodic advertising has been successfully synced.
This callback notifies the application that the periodic advertising set has been successfully synced, 
and will now start to receive periodic advertising reports.*/
void synced_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info){

}
/*The periodic advertising sync has been terminated.
This callback notifies the application that the periodic advertising sync has been terminated, either by local request,
remote request or because due to missing data, e.g. by being out of range or sync.*/
void term_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_term_info *info){

}
/*Periodic advertising data received.
This callback notifies the application of an periodic advertising report.*/
void recv_cb(struct bt_le_per_adv_sync *sync, const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf){
    gpio_pin_set(dev, PIN, (int)led_is_on);//turn on and off the led for received data
    led_is_on = !led_is_on;
}

const struct bt_le_per_adv_sync_cb cb = {
    .synced = synced_cb,
    .term = term_cb,
    .recv = recv_cb,
};


void main(void)
{
    struct device *dev; //for led
	bool led_is_on = false;
	dev = device_get_binding(LED0); //for led0
	gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);//configure

	struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = 0x0010,
		.window     = 0x0010,
	};
	int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = bt_le_per_adv_sync_create(&sync_param, &cb, struct bt_le_per_adv_sync **out_sync)

	err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		return;
	}

	do {
		k_sleep(K_MSEC(400));

		/* Start advertising */
		err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
				      NULL, 0);
		if (err) {
			printk("Advertising failed to start (err %d)\n", err);
			return;
		}

		k_sleep(K_MSEC(400));

		err = bt_le_adv_stop();
		if (err) {
			printk("Advertising failed to stop (err %d)\n", err);
			return;
		}
	} while (1);
}