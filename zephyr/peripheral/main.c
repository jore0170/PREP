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
#include <sys/byteorder.h>
#include <zephyr.h>

#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>

#include "cts.h"

#define CONFIG_BT_GATT_DYNAMIC_DB

/* Custom Service Variables */
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


//static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
//			void *buf, uint16_t len, uint16_t offset)
//{
//	const char *value = attr->user_data;

//	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
//				 strlen(value));
//}

//static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
//			 const void *buf, uint16_t len, uint16_t offset,
//			 uint8_t flags)
//{
//	uint8_t *value = attr->user_data;

//	if (offset + len > sizeof(vnd_value)) {
//		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
//	}

//	memcpy(value + offset, buf, len);

//	return len;
//}

static uint8_t simulate_vnd;
static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			uint8_t err)
{
	printk("Indication %s\n", err != 0U ? "fail" : "success");
	indicating = 0U;
}

#define MAX_DATA 74
static uint8_t vnd_long_value[] = {
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '1',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '2',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '3',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '4',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '5',
		  'V', 'e', 'n', 'd', 'o', 'r', ' ', 'd', 'a', 't', 'a', '6',
		  '.', ' ' };

static ssize_t read_long_vnd(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr, void *buf,
			     uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(vnd_long_value));
}

static ssize_t write_long_vnd(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr, const void *buf,
			      uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset + len > sizeof(vnd_long_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_gatt_cep vnd_long_cep = {
	.properties = BT_GATT_CEP_RELIABLE_WRITE,
};

static int signed_value;

static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(signed_value));
}

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset,
			    uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > sizeof(signed_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

static ssize_t write(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset){
			     //return bt_gatt_attr_write_ccc(conn, attr, buf, len, offset, BT_GATT_WRITE_FLAG_CMD);//flag for indicating write operation is a command (write without response )
			     uint8_t *value = attr->user_data;
                             memcpy(value + offset, buf, len);
                             return len;
                                        }

static ssize_t read(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset){
	uint8_t *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(value));
				 }

static const struct bt_uuid_128 vnd_signed_uuid = BT_UUID_INIT_128(
	0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x13,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x13);

static const struct bt_uuid_128 vnd_write_cmd_uuid = BT_UUID_INIT_128(
	0xf4, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

//static ssize_t write_without_rsp_vnd(struct bt_conn *conn,
//				     const struct bt_gatt_attr *attr,
//				     const void *buf, uint16_t len, uint16_t offset,
//				     uint8_t flags)
//{
//	uint8_t *value = attr->user_data;

	/* Write request received. Reject it since this char only accepts
	 * Write Commands.
	 */
//	if (!(flags & BT_GATT_WRITE_FLAG_CMD)) {
//		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
//	}

//	if (offset + len > sizeof(vnd_value)) {
//		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
//	}

//	memcpy(value + offset, buf, len);

//	return len;
//}

static struct bt_gatt_attr custom_attrs[] = {
	/* Custom Primary Service Declaration */
	BT_GATT_PRIMARY_SERVICE(&service_uuid),

	BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read, write, NULL),

};

//static struct bt_gatt_service custom_svc = BT_GATT_SERVICE(custom_attrs);

/* Custom Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(custom_svc,
	BT_GATT_PRIMARY_SERVICE(&service_uuid),
	/* Capabilities: Readable only when unlocked. Never writable. */
	BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid, 
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, 
						    read,  write, NULL)//these callback functions are implemented in gatt.h?
				   );

static const struct bt_data ad[] = {
	//BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	//BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      //0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		0x5b, 0xd9, 0xa0, 0x01, 0x9b, 0x08, 0xfa, 0xaa,
		0xa4, 0x46, 0xfa, 0x34, 0xfd, 0x2a, 0xff, 0x93),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	cts_init();

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static void bas_notify(void)
{
	uint8_t battery_level = bt_gatt_bas_get_battery_level();

	battery_level--;

	if (!battery_level) {
		battery_level = 100U;
	}

	bt_gatt_bas_set_battery_level(battery_level);
}

static void hrs_notify(void)
{
	static uint8_t heartrate = 90U;

	/* Heartrate measurements simulation */
	heartrate++;
	if (heartrate == 160U) {
		heartrate = 90U;
	}

	bt_gatt_hrs_notify(heartrate);
}

void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	//bt_conn_cb_register(&conn_callbacks);
	//bt_conn_auth_cb_register(&auth_cb_display);

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		k_sleep(K_SECONDS(1));

		/* Current Time Service updates only when time is changed */
		//cts_notify();

		/* Heartrate measurements simulation */
		//hrs_notify();

		/* Battery level simulation */
		//bas_notify();

		/* Vendor indication simulation */
		if (simulate_vnd) {
			if (indicating) {
				continue;
			}

			//ind_params.attr = &vnd_svc.attrs[2];
			ind_params.func = indicate_cb;
			ind_params.data = &indicating;
			ind_params.len = sizeof(indicating);

			if (bt_gatt_indicate(NULL, &ind_params) == 0) {
				indicating = 1U;
			}
		}
	}
}