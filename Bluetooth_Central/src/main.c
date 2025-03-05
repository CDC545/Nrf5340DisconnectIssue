/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include "bluetooth.h"
#include <zephyr/shell/shell.h>
#include <zephyr/settings/settings.h>  // Correct include path
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Add these declarations at the top of main.c after the includes
// extern SensorInstance sensor_instances[MAX_SENSORS];
// extern BluetoothController received_controller;
// extern bool notifications_enabled;
// extern int num_sensor_instances;  // This one isn't needed anymore

LOG_MODULE_REGISTER(Main, LOG_LEVEL_INF);

#define USER_BUTTON DK_BTN1_MSK
#define SCAN_LED DK_LED1
#define CONNECTED_LED DK_LED2
#define DEVICE_LIST_PRINT_INTERVAL_MS 2000  // Print every 2 seconds

static bool scanning = false;
static bool button_pressed = false;
static struct k_work_delayable print_work;



typedef enum {
    SENSOR_DATA_TYPE_INT,
    SENSOR_DATA_TYPE_FLOAT,
    SENSOR_DATA_TYPE_DOUBLE
    // Add more data types as needed
} SensorDataType;


// Add this function to handle periodic printing
static void print_work_handler(struct k_work *work)
{
	if (scanning) {
		LOG_INF("\nCurrently found devices:");
		print_found_devices();
		// Schedule next print
		k_work_schedule(&print_work, K_MSEC(DEVICE_LIST_PRINT_INTERVAL_MS));
	}
}

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	bool pressed = button_state & USER_BUTTON;
	bool button_change = has_changed & USER_BUTTON;

	if (button_change && pressed) {
		if (!button_pressed) {
			// First press starts scanning
			LOG_INF("Starting scan...");
			scanning = true;
			start_scan();
			button_pressed = true;
			// Start periodic printing
			k_work_schedule(&print_work, K_MSEC(DEVICE_LIST_PRINT_INTERVAL_MS));
		} else {
			// Second press stops scanning
			scanning = false;
			bt_le_scan_stop();
			LOG_INF("\nScanning stopped. Final device list:");
			print_found_devices();
			LOG_INF("\nPress button again to connect to device 1, or wait for timeout to scan again");
			button_pressed = false;
		}
	}
}

static void on_connected(void) {
	scanning = false;
	dk_set_led(SCAN_LED, 0);
	dk_set_led(CONNECTED_LED, 1);
	LOG_INF("Connected to peripheral device");
}

static void on_disconnected(void) {
	dk_set_led(CONNECTED_LED, 0);
	LOG_INF("Disconnected from peripheral device");
}

static void data_received(const uint8_t *data, uint16_t len)
{
	// Only process the data, don't print it
	// Process your data here...
	
	// Optionally, you can still print specific events or conditions
	// if (some_important_condition) {
	//     LOG_INF("Important data received!");
	// }
}

static int init_button_and_leds(void)
{
	int err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
		return err;
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
		return err;
	}

	return 0;
}

// Add this function to handle the connect command
static int cmd_connect(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "Usage: connect <device_number>");
		return -EINVAL;
	}

	int device_number = atoi(argv[1]);
	if (device_number <= 0 || device_number > get_num_found_devices()) {
		shell_error(shell, "Invalid device number. Choose 1-%d", get_num_found_devices());
		return -EINVAL;
	}

	// Stop scanning before attempting to connect
	if (scanning) {
		scanning = false;
		bt_le_scan_stop();
	}

	// Add a small delay to ensure scanning has stopped
	k_sleep(K_MSEC(100));

	shell_print(shell, "Connecting to device %d...", device_number);
	int err = connect_to_device(device_number);
	if (err) {
		shell_error(shell, "Connection failed (err %d). Device might not be connectable.", err);
	}
	return err;
}

// Add a disconnect command
static int cmd_disconnect(const struct shell *shell, size_t argc, char **argv)
{
	disconnect_current_connection();
	shell_print(shell, "Disconnected");
	return 0;
}

// Add a scan command
static int cmd_scan(const struct shell *shell, size_t argc, char **argv)
{
	if (scanning) {
		scanning = false;
		bt_le_scan_stop();
		shell_print(shell, "Scanning stopped");
	} else {
		scanning = true;
		start_scan();
		shell_print(shell, "Scanning started");
		// Start periodic printing
		k_work_schedule(&print_work, K_MSEC(DEVICE_LIST_PRINT_INTERVAL_MS));
	}
	return 0;
}

// Add these command handlers
static int cmd_enable(const struct shell *shell, size_t argc, char **argv)
{
	int err = enable_notifications();
	if (err) {
		if (err == -EAGAIN) {
			LOG_INF("Security not established yet, please try again in a moment");
		} else {
			LOG_ERR("Failed to enable notifications (err %d)", err);
		}
	} else {
		LOG_INF("Notifications enabled");
	}
	return err;
}

static int cmd_disable(const struct shell *shell, size_t argc, char **argv)
{
	int err = disable_notifications();
	if (err) {
		shell_error(shell, "Failed to disable notifications (err %d)", err);
	} else {
		shell_print(shell, "Notifications disabled");
	}
	return err;
}

// Add near the top with other forward declarations
static void list_bonds_callback(const struct bt_bond_info *info, void *user_data);

// Add this at the top with other static variables
static int bond_count = 0;

// Update the callback to count bonds
static void list_bonds_callback(const struct bt_bond_info *info, void *user_data)
{
	bt_addr_le_t *addrs = user_data;
	
	if (bond_count < CONFIG_BT_MAX_PAIRED) {
		bt_addr_le_copy(&addrs[bond_count], &info->addr);
		bond_count++;
	}
}

// Update the bonds command handler
static int cmd_bonds(const struct shell *shell, size_t argc, char **argv)
{
    int err = bluetooth_list_bonds();
    return err;
}

// Update the forget command handler
static int cmd_forget(const struct shell *shell, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(shell, "Usage: bt forget <index>");
		return -EINVAL;
	}

	int index = atoi(argv[1]);
	bt_addr_le_t addrs[CONFIG_BT_MAX_PAIRED];
	
	// Reset bond count
	bond_count = 0;
	
	// Get list of bonds
	bt_foreach_bond(BT_ID_DEFAULT, list_bonds_callback, addrs);

	if (index < 0 || index >= bond_count) {
		shell_error(shell, "Invalid index. Choose 0-%d", bond_count - 1);
		return -EINVAL;
	}

	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(&addrs[index], addr, sizeof(addr));

	int err = bt_unpair(BT_ID_DEFAULT, &addrs[index]);
	if (err) {
		shell_error(shell, "Failed to remove bond (err %d)", err);
		return err;
	}

	shell_print(shell, "Removed bond for device %s", addr);
	return 0;
}

// Add this command handler
static int cmd_rename(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_error(shell, "Usage: bt rename <device_name> <new_name>");
        return -EINVAL;
    }

    int err = send_rename_command(argv[1], argv[2]);
    if (err) {
        shell_error(shell, "Failed to send rename command (err %d)", err);
    } else {
        shell_print(shell, "Rename command sent successfully");
    }
    return err;
}

// Update the shell command handler
static int cmd_sensors(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        shell_error(shell, "Usage: bt sensors <device_name>");
        return -EINVAL;
    }

    int err = request_sensor_info_command(argv[1]);
    if (err) {
        shell_error(shell, "Failed to request sensor info (err %d)", err);
        return err;
    }

    shell_print(shell, "Sensor info request sent successfully");
    return 0;
}

// Add this command handler
static int cmd_list_sensors(const struct shell *shell, size_t argc, char **argv)
{
    int err = print_sensor_instances();
    if (err) {
        shell_error(shell, "Failed to print sensor instances (err %d)", err);
    }
    return err;
}

// Add this command handler
static int cmd_stream(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 3) {
        shell_error(shell, "Usage: bt stream <device_name> <on|off>");
        return -EINVAL;
    }

    // Convert on/off to enable_stream value
    uint8_t enable_stream;
    if (strcmp(argv[2], "on") == 0) {
        enable_stream = 0x01;
    } else if (strcmp(argv[2], "off") == 0) {
        enable_stream = 0x00;
    } else {
        shell_error(shell, "Invalid option. Use 'on' or 'off'");
        return -EINVAL;
    }

    int err = request_sensor_data_stream(argv[1], enable_stream);
    if (err) {
        shell_error(shell, "Failed to request data stream (err %d)", err);
        return err;
    }

    shell_print(shell, "Data stream request sent successfully");
    return 0;
}

// Finally the shell command registration
SHELL_STATIC_SUBCMD_SET_CREATE(sub_bluetooth,
	SHELL_CMD(connect, NULL, "Connect to device by number (connect <number>)", cmd_connect),
	SHELL_CMD(disconnect, NULL, "Disconnect current connection", cmd_disconnect),
	SHELL_CMD(scan, NULL, "Start/stop scanning", cmd_scan),
	SHELL_CMD(enable, NULL, "Enable notifications", cmd_enable),
	SHELL_CMD(disable, NULL, "Disable notifications", cmd_disable),
	SHELL_CMD(bonds, NULL, "Show bonded devices", cmd_bonds),
	SHELL_CMD(forget, NULL, "Remove bond (forget <index>)", cmd_forget),
	SHELL_CMD(rename, NULL, "Rename device (rename <device_name> <new_name>)", cmd_rename),
	SHELL_CMD(sensors, NULL, "Request sensor information (sensors <device_name>)", cmd_sensors),
	SHELL_CMD(list, NULL, "List all sensor instances", cmd_list_sensors),
	SHELL_CMD(stream, NULL, "Request sensor data stream (stream <device_name> <on|off>)", cmd_stream),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(bt, &sub_bluetooth, "Bluetooth commands", NULL);

int main(void)
{
	int err;

	// Initialize settings first
	err = settings_subsys_init();
	if (err) {
		LOG_ERR("Failed to initialize settings subsystem (err %d)", err);
		return 0;
	}

	// Load settings before any other initialization
	err = settings_load();
	if (err) {
		LOG_ERR("Failed to load settings (err %d)", err);
		return 0;
	}

	// Add a delay after settings load
	k_sleep(K_MSEC(100));

	LOG_INF("Settings loaded successfully");

	// Initialize buttons and LEDs
	err = init_button_and_leds();
	if (err) {
		LOG_ERR("Button and LED init failed (err %d)", err);
		return 0;
	}

	// Initialize Bluetooth with callbacks
	struct bluetooth_callbacks callbacks = {
		.connected = on_connected,
		.disconnected = on_disconnected,
		.data_received = data_received,
	};

	err = bluetooth_init(&callbacks);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	LOG_INF("Central device started. Press Button 1 to start scanning");

	// Initialize work queue for periodic printing
	k_work_init_delayable(&print_work, print_work_handler);

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
