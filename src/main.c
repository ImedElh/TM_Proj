/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/logging/log.h>
// GPIO driver header
#include <zephyr/drivers/gpio.h>
#include "remote.h"
// Bonding headers
//#include <zephyr/settings/settings.h>

//  Logging module registration
#define LOG_MODULE_NAME app
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// Get led0 debugging led 
#define DEBUG_LED DT_NODELABEL(led0)
static const struct gpio_dt_spec debugLed = GPIO_DT_SPEC_GET(DEBUG_LED, gpios);


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
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey);
static void auth_cancel(struct bt_conn *conn);

struct bt_conn_cb bluetooth_callbacks = {
	.connected 		= on_connected,
	.disconnected 	= on_disconnected,
	.security_changed = on_security_changed,
};
static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};
struct bt_remote_service_cb remote_callbacks = {
	.notif_changed = on_notif_changed,
    .data_received = on_data_received,
};



/*Security  Callbacks */

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
 // Autentication callback to pass to security level 3 or 4
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Passkey for %s: %06u\n", addr, passkey);
}

// This will let us know when the pairing has been cancelled.
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing cancelled: %s\n", addr);
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
	int err, ret;
    printk("Starting Bluetooth Peripheral LBS example\n");
	LOG_INF("Hello World! %s\n", CONFIG_BOARD);

	if (!device_is_ready(debugLed.port)) {
	  return -1;
	}
	
	ret = gpio_pin_configure_dt(&debugLed, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return -1;
	}
   // Active the debug led
   // gpio_pin_set_dt(&debugLed,1);

#ifdef LOW_LEVEL_I2C // LOW level I2C transaction
   uint8_t reading[3] = {0} ;
   uint8_t sensor_regs[2] ={0x01,0x0F}; // register number
   err = i2c_write_read_dt(&dev_i2c,&sensor_regs[0],2,&reading[0],3);
   	if (err) {
		LOG_ERR("Error I2C Comm %d", err);
	}
#endif

#ifdef BLE_BONDING
	settings_load(); //After resetting the peripheral, so that previous bonds can be restored.
	bt_unpair(); //should be called to erase all bonded devices
#endif
	// Enable bluetooth communication
    err = bluetooth_init(&bluetooth_callbacks, &remote_callbacks);
    if (err) {
        LOG_ERR("bt_enable returned %d", err);
    }
    // Register the authentication callbacks
	// Authentication for MITM attack protection
	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
	LOG_INF("Failed to register authorization callbacks.\n");
	return; 
    }
	LOG_INF("Running...");
	// Start sensor simulator timer
	k_timer_start(&update_timer, K_SECONDS(1),K_SECONDS(1));
	getAcc0Data();
	for (;;) {
		LOG_INF("Hello, I am main\n");
		k_sleep(K_MSEC(1000));
	}
}
