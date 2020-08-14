/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>
#include <bluetooth/conn.h>


#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/*
 * Set Advertisement data. Based on the Eddystone specification:
 * https://github.com/google/eddystone/blob/master/protocol-specification.md
 * https://github.com/google/eddystone/tree/master/eddystone-url
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
	BT_DATA_BYTES(BT_DATA_SVC_DATA16,
		      0xaa, 0xfe, /* Eddystone UUID */
		      0x10, /* Eddystone-URL frame type */
		      0x00, /* Calibrated Tx power at 0m */
		      0x00, /* URL Scheme Prefix http://www. */
		      'z', 'e', 'p', 'h', 'y', 'r',
		      'p', 'r', 'o', 'j', 'e', 'c', 't',
		      0x08), /* .org */
	/* CUSTOM Service UUID 93ff2afd-34fa-46a4-aafa-089b01a0d95b */
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
	0x5b, 0xd9, 0xa0, 0x01, 0x9b, 0x08, 0xfa, 0xaa,
	0xa4, 0x46, 0xfa, 0x34, 0xfd, 0x2a, 0xff, 0x93),
};
////////////////////////////////////////////////////////////////////////////////////////////////////
//try just altering ad with custom service and characteristic that includes a read/write capability
//use BT_GATT_PRIMARY_SERVICE() and BT_GATT_CHARACTERISTIC() to define and bt_gatt_service_register()
//to register
////////////////////////////////////////////////////////////////////////////////////////////////////


/* Service UUID a3c87500-8ed3-4bdf-8a39-a01bebede295 */
//93ff2afd-34fa-46a4-aafa-089b01a0d95b from online GUID generator
static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
	0x5b, 0xd9, 0xa0, 0x01, 0x9b, 0x08, 0xfa, 0xaa,
	0xa4, 0x46, 0xfa, 0x34, 0xfd, 0x2a, 0xff, 0x93);

/* Characteristic UUID a3c87501-8ed3-4bdf-8a39-a01bebede295 */
//2d769eb9-d2d4-4810-b605-b8d10ceeb6bf from online GUID generator
static struct bt_uuid_128 custom_char_uuid = BT_UUID_INIT_128(
	0xbf, 0xb6, 0xee, 0x0c, 0xd1, 0xb8, 0x05, 0xb6,
	0x10, 0x48, 0xd4, 0xd2, 0xb9, 0x9e, 0x76, 0x2d);

//use bt_gatt_read_func_t typedef or ssize_t?
static ssize_t read(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset){
	uint8_t *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(eds_ecdh));
				 }

//what about bt_gatt_attr_write_ccc()?
static ssize_t write(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset){

						return bt_gatt_attr_write_ccc(*conn, *attr, *buf, len, offset, BT_GATT_WRITE_FLAG_CMD);//flag for indicating write operation is a command (write without response)
					}
//check out bt_gatt_write_without_response_cb()


BT_GATT_SERVICE_DEFINE(eds_svc,
	BT_GATT_PRIMARY_SERVICE(&service_uuid),
	/* Capabilities: Readable only when unlocked. Never writable. */
	BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid, 
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
						    read,  write, NULL)//these callback functions are implemented in gatt.h?
				   );
/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	/* Start advertising */
    //bt_le_adv_start originally had BT_LE_ADV_NCONN
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Beacon started\n");
}

void main(void)
{
	int err;

	printk("Starting Beacon Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
}