/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr.h>
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

#define BT_LE_PER_ADV_OPT_NONE 0

struct bt_le_per_adv_param{
	uint16_t interval_min;
	uint16_t interval_max;
	uint32_t options;
};

struct bt_le_ext_adv adv;

static uint8_t mfg_data[] = { 0xff, 0xff, 0x00 };

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, 3),
};

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	mfg_data[2]++;
}

void main(void)
{
	struct device *dev; //for led
	bool led_is_on = false;
	dev = device_get_binding(LED0); //for led0
	gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);//configure

	const struct bt_le_adv_param adv_param = {
		.id = 0x01, //local identity
		.sid = 0x01, //Advertising set identifier
		.secondary_max_skip = 0x01, //max events advertiser can skip before sending data on secondary channel
		.options = BT_LE_ADV_OPT_EXT_ADV ,//bit mask has other options to try
		.interval_min = 0x18, //0x18 = decimal 24 -> (24 * 0.625) = 15ms
		.interval_max = 0x18,
		.peer = NULL, //set for directed advertising
	};

	void sent_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_sent_info *info){

	}

	void connected_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_connected_info *info){

	}

	void scanned_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_scanned_info *info){

	}

	const struct bt_le_ext_adv_cb cb = {
		.sent = sent_cb,
		.connected = connected_cb,
		.scanned = scanned_cb,

	};

    struct bt_le_ext_adv_start_param start_param = {
        .timeout = 0, //zero for no timeout
        .num_events = 0, //zero for no event limit
    };

	const struct bt_le_per_adv_param per_adv_param = {
		.interval_min = 0x18, //set the same as extended params
		.interval_max = 0x18,
		.options = BT_LE_PER_ADV_OPT_NONE, //this is a convenience value for no options, other option is setting tx power
	};

	struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_NONE | BT_LE_ADV_OPT_USE_IDENTITY,
		.interval   = 0x0010,
		.window     = 0x0010,
	};
	int err;

	printk("Starting Scanner/Advertiser Demo\n");
	/*Set public address for controller. */
	//void bt_ctlr_set_public_addr(const uint8_t *addr);
	
	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful bt init
		return;
	}

	printk("Bluetooth initialized\n");
	
	/*err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		printk("Starting scanning failed (err %d)\n", err);
		gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful scan start

		return;
	}*/


	do {
		k_sleep(K_MSEC(400));
		/*create advertising set for extended advertising*/
		err = bt_le_ext_adv_create(&adv_param, &cb, **adv);//adv is output advertising set object
		if (err) {
			gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful creation
		return;
	    }
		/*Set the extended advertising parameters*/
		err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) {
			gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful set
		return;
	    }
		/*Start extended advertising*/
		err = bt_le_ext_adv_start(adv, &start_param);
        if (err) {
			gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful start
		return;
	    }
		/*set periodic advertisement parameters*/
		err = bt_le_per_adv_set_param(adv, per_adv_param);
        if (err) {
			gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful set
		return;
	    }
		/*setting periodic advertisement data*/
		err = bt_le_per_adv_set_data(adv, ad, ARRAY_SIZE(ad));
        if (err) {
			gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful set
		return;
	    }
		/*Start periodic advertising*/
 		err = bt_le_per_adv_start(adv);
		if (err) {
			gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful start
		return;
	    }
		/*Create periodic advertising sync object*/
		//int bt_le_per_adv_sync_create(const struct bt_le_per_adv_sync_param *param, const struct bt_le_per_adv_sync_cb *cb, struct bt_le_per_adv_sync **out_sync)

		/* Start advertising */
		err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
				      NULL, 0);
		if (err) {
			gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful adv start
			printk("Advertising failed to start (err %d)\n", err);
			return;
		}

		k_sleep(K_MSEC(40000));//was 400
		gpio_pin_set(dev, PIN, (int)led_is_on);//turn off led0 indicating unsuccesful adv start


		err = bt_le_adv_stop();
		if (err) {
			printk("Advertising failed to stop (err %d)\n", err);
			return;
		}
	} while (1);
}