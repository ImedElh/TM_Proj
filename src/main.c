/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include "remote.h"

#define LOG_MODULE_NAME app
LOG_MODULE_REGISTER(LOG_MODULE_NAME);


static void update_timer_handler(struct k_timer *timer_id);

K_TIMER_DEFINE(update_timer, update_timer_handler, NULL);


static struct bt_conn *current_conn;
static bool isNotifEnabled = false;

/* Declarations */
void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_notif_changed(enum bt_button_notifications_enabled status);
void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
static void on_security_changed(struct bt_conn *conn, bt_security_t level,enum bt_security_err err);

struct bt_conn_cb bluetooth_callbacks = {
	.connected 		= on_connected,
	.disconnected 	= on_disconnected,
	.security_changed = on_security_changed,
};
struct bt_remote_service_cb remote_callbacks = {
	.notif_changed = on_notif_changed,
    .data_received = on_data_received,
};

/* Callbacks */

static void on_security_changed(struct bt_conn *conn, bt_security_t level,enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u\n", addr, level);
	} else {
		LOG_INF("Security failed: %s level %u err %d\n", addr, level,
			err);
	}
}


void on_connected(struct bt_conn *conn, uint8_t err)
{
	if(err) {
		LOG_ERR("connection err: %d", err);
		return;
	}
	LOG_INF("Connected.");
	current_conn = bt_conn_ref(conn);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason: %d)", reason);
	if(current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

void on_notif_changed(enum bt_button_notifications_enabled status)
{
	if (status == BT_BUTTON_NOTIFICATIONS_ENABLED) {
		isNotifEnabled = true;
		//LOG_INF("Notifications enabled");
	}
	else {
		isNotifEnabled = false;
		//LOG_INF("Notificatons disabled");
	}
}

void on_data_received(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	uint8_t temp_str[len+1];
	memcpy(temp_str, data, len);
	temp_str[len] = 0x00;

	//LOG_INF("Received data on conn %p. Len: %d", (void *)conn, len);
	//LOG_INF("Data: %s", log_strdup(temp_str));
	LOG_INF("Data: %s", (char*)temp_str);

}

void simulate_sensor(void){
	static uint16_t sensor_value = 0;
	sensor_value++;
	if(sensor_value == 1000){
	  sensor_value = 0;	
	}
	  int err = 0;

		set_sensor(sensor_value);
		if(isNotifEnabled) {
		err = send_button_notification(current_conn, sensor_value, sizeof(sensor_value));
		}
		if (err) {
			LOG_ERR("couldn't send notification (err: %d)", err);
		}
}

static void update_timer_handler(struct k_timer *timer_id)
{
    //LOG_INF("Timer expired in handler");
	simulate_sensor();
}


/* main */

void main(void)
{
	int err;
    printk("Starting Bluetooth Peripheral LBS example\n");
	LOG_INF("Hello World! %s\n", CONFIG_BOARD);

	err = bluetooth_init(&bluetooth_callbacks, &remote_callbacks);
	if (err) {
		LOG_ERR("bt_enable returned %d", err);
	}
	LOG_INF("Running...");
	// Start sensor simulator timer
	k_timer_start(&update_timer, K_SECONDS(1),K_SECONDS(1));
	for (;;) {
		
		k_sleep(K_MSEC(1000));
	}
}
