/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <stdlib.h>
#include <time.h>
#include "bluetooth.h"
#include <zephyr/random/random.h>  // for sys_csrand_get()
#include <zephyr/shell/shell.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(Main, LOG_LEVEL_INF);

#define USER_BUTTON DK_BTN1_MSK
#define SEND_RATE_MS 8    // 8ms to match connection interval (7.5ms + margin)

// Simulated sensor ranges
#define TEMP_MIN 20
#define TEMP_MAX 30
#define HUMIDITY_MIN 40
#define HUMIDITY_MAX 60
#define PRESSURE_MIN 980
#define PRESSURE_MAX 1020

#define MAX_SENSORS 10

#define UNIQUE_ID_LENGTH 10  // 10 bytes = 80 bits

// Define sensor details structure
struct sensor_details {
    uint8_t id;
    uint16_t type;
    uint16_t data_rate;
    bool connected;
};

// Global controller and metadata arrays
BluetoothController controller;
SensorInstance sensor_instances[MAX_SENSORS];
static SensorMetaData sensor_metadata[MAX_SENSORS];
static uint8_t sensor_index = 0;  // Global index counter for sensors

// Function declarations
uint64_t generate_unique_id(void);
static void update_metadata(uint8_t *payload);

//--------------------------------------------------------------
// Sensor Lookup Table
//--------------------------------------------------------------
typedef struct {
    int id;                  // Unique sensor identifier
    const char *name;        // Human-readable sensor name
    const char *units;       // Measurement units
    int defaultDataRate;     // Default data rate in Hz
} SensorLookup;

// Create a lookup table that maps sensor numbers to sensor parameters.
static const SensorLookup sensor_lookup_table[] = {
    {1, "Temperature",  "°C",  10},
    {2, "Pressure",     "psi", 10},
    {3, "Force",        "N",   10},
    {4, "RPM",          "rpm", 50},
    {5, "Speed",        "km/h",50},
    {6, "Voltage",      "V",   10},
    {7, "Current",      "A",   10},
    {8, "Acceleration", "m/s²",100},
    // Optionally, add an "unknown" entry for sensor IDs that do not match.
    {0, "Unknown",      "",    0}
};

#define NUM_SENSORS (sizeof(sensor_lookup_table) / sizeof(sensor_lookup_table[0]))

// Function to get sensor info from lookup table
static const SensorLookup* get_sensor_info(uint16_t sensor_type) {
    for (int i = 0; i < ARRAY_SIZE(sensor_lookup_table); i++) {
        if (sensor_lookup_table[i].id == sensor_type) {
            return &sensor_lookup_table[i];
        }
    }
    // Return the "Unknown" entry if no match found
    return &sensor_lookup_table[ARRAY_SIZE(sensor_lookup_table) - 1];
}

// Sensor management functions
static void create_sensor_instance(int index, uint16_t sensor_type, uint8_t reading_size, 
                                 uint8_t dim_width, uint8_t dim_height,
                                 uint8_t data_type, uint16_t data_rate) {
    // Setup the metadata
    sensor_metadata[index].uniqueId = generate_unique_id();
    sensor_metadata[index].sensorType = sensor_type;
    sensor_metadata[index].readingSize = reading_size;
    sensor_metadata[index].dimensionWidth = dim_width;
    sensor_metadata[index].dimensionHeight = dim_height;
    sensor_metadata[index].dataType = data_type;
    sensor_metadata[index].dataRate = data_rate;

    // Setup the instance
    sensor_instances[index].instanceId = index + 1;
    sensor_instances[index].sensorReading = 0.0f;
    sensor_instances[index].unixTime = k_uptime_get_32() / 1000;
    sensor_instances[index].meta = &sensor_metadata[index];
    sensor_instances[index].controller = &controller;

    // Update global index if this is highest seen
    if (index >= sensor_index) {
        sensor_index = index + 1;
        controller.numberOfSensors = sensor_index;
    }

    LOG_INF("Created sensor instance %d: Type %d", 
            sensor_instances[index].instanceId,
            sensor_metadata[index].sensorType);
}

// Function to generate simulated sensor data
static void update_metadata(uint8_t *payload) {
    // Verify actual size needed
    size_t actual_size = sizeof(BluetoothController) + 
                        (controller.numberOfSensors * sizeof(SensorMetaData));
    
    if (actual_size > 247) {  // BLE MTU size limit
        LOG_ERR("Metadata too large: %d bytes", actual_size);
		return;
    }

    size_t offset = 0;
    
    // Add controller metadata
    memcpy(&payload[offset], &controller, sizeof(BluetoothController));
    offset += sizeof(BluetoothController);
    
    // Add sensor metadata
    for (int i = 0; i < sensor_index; i++) {
        memcpy(&payload[offset], &sensor_metadata[i], sizeof(SensorMetaData));
        offset += sizeof(SensorMetaData);
    }

    LOG_DBG("Updated metadata packet size: %d bytes", offset);
}

// Calculate actual needed size
#define METADATA_SIZE (sizeof(BluetoothController) + \
                     (MAX_SENSORS * sizeof(SensorMetaData)))

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	bool button_pressed = button_state & USER_BUTTON;
	bool button_changed = has_changed & USER_BUTTON;

	if (button_changed && button_pressed) {
			send_data_enabled = !send_data_enabled;
		LOG_INF("Data sending %s", send_data_enabled ? "enabled" : "disabled");
	}
}

static int init_button(void)
{
	int err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
		return err;
	}
	return 0;
}

static uint32_t packet_count = 0;
static uint32_t last_print = 0;
static uint32_t last_conn_info = 0;

static void print_conn_info(struct bt_conn *conn) {
	struct bt_conn_info info;
	int err = bt_conn_get_info(conn, &info);

	if (err) {
		LOG_ERR("Failed to get connection info (err %d)", err);
		return;
	}

	// Calculate actual connection interval in milliseconds
	int interval_ms = (int)(info.le.interval * 1.25);

	LOG_INF("Connection parameters:");
	LOG_INF("  Interval: %d ms (%d units)", interval_ms, info.le.interval);
	LOG_INF("  Latency: %d intervals", info.le.latency);
	LOG_INF("  Timeout: %d ms", info.le.timeout * 10);

	// Get current MTU
	uint16_t mtu = bt_gatt_get_mtu(conn);
	LOG_INF("  MTU: %d bytes", mtu);

	// PHY and Data Length will be logged in their respective callbacks
	// when they are updated by the central
}

static void name_threads(void) {
	k_thread_name_set(k_current_get(), "main");
}

static int update_controller_info(const char* name, const char* address, 
                                uint16_t type, float firmware_ver, 
                                float battery_level, uint8_t status) {
    // Update controller information
    strncpy(controller.controllerName, name, sizeof(controller.controllerName));
    strncpy(controller.controllerAddress, address, sizeof(controller.controllerAddress));
    controller.controllerType = type;
    controller.firmwareVersion = firmware_ver;
    controller.batteryLevel = battery_level;
    controller.status = status;
    // Note: numberOfSensors is managed by add_sensor/remove_sensor

    LOG_INF("Updated controller: Name=%s, Address=%s", name, address);
    LOG_INF("  Type: 0x%04x, FW: %.2f, Battery: %.1f%%, Status: 0x%02x",
            type, firmware_ver, battery_level, status);
    LOG_INF("  Number of sensors: %d", controller.numberOfSensors);

    return 0;
}

// Function to update only battery level
static inline void update_battery_level(float battery_level) {
    controller.batteryLevel = battery_level;
    LOG_DBG("Battery level updated: %.1f%%", battery_level);
}

// Function to simulate battery drain (for demonstration)
static void simulate_battery_drain(void) {
    static uint32_t last_battery_update = 0;
    static const uint32_t BATTERY_UPDATE_INTERVAL = 60000;  // Update every minute
    
    uint32_t now = k_uptime_get_32();
    if (now - last_battery_update >= BATTERY_UPDATE_INTERVAL) {
        // Simulate 0.1% battery drain per minute
        if (controller.batteryLevel > 0.1f) {
            update_battery_level(controller.batteryLevel - 0.1f);
        }
        last_battery_update = now;
    }
}

uint64_t generate_unique_id(void)
{
    uint64_t id = 0;
    int ret = sys_csrand_get((uint8_t *)&id, sizeof(id));
    if (ret != 0) {
        printk("Error generating random ID: %d\n", ret);
        // Depending on your application, you may wish to handle the error differently.
        return 0;
    }
    return id;
}

static int cmd_print_sensors(const struct shell *shell, size_t argc, char **argv)
{
    LOG_INF("=== Sensor Instances Overview ===");
    LOG_INF("Controller: %s (%s)", 
            controller.controllerName, 
            controller.controllerAddress);
    LOG_INF("Controller Status: FW %.2f, Battery %.1f%%, Status 0x%02x",
            controller.firmwareVersion,
            controller.batteryLevel,
            controller.status);
    
    for (int i = 0; i < sensor_index; i++) {
        const SensorLookup* sensor_info = get_sensor_info(sensor_instances[i].meta->sensorType);
        LOG_INF("----------------------------");
        LOG_INF("Instance %d:", sensor_instances[i].instanceId);
        LOG_INF("  Sensor ID: %016llx", sensor_instances[i].meta->uniqueId);
        LOG_INF("  Type: %s (%d)", sensor_info->name, sensor_instances[i].meta->sensorType);
        LOG_INF("  Dimensions: %dx%d", 
               sensor_instances[i].meta->dimensionWidth,
               sensor_instances[i].meta->dimensionHeight);
        LOG_INF("  Data Type: %d", sensor_instances[i].meta->dataType);
        LOG_INF("  Reading: %.2f %s (at %ld)", 
               sensor_instances[i].sensorReading,
               sensor_info->units,
               sensor_instances[i].unixTime);
        LOG_INF("  Data Rate: %d Hz", sensor_instances[i].meta->dataRate);
    }
    LOG_INF("==============================");
    
    return 0;
}

static int cmd_rename_device(const struct shell *shell, size_t argc, char **argv)
{
    if (argc != 2) {
        LOG_ERR("Usage: sensors rename <new_name>");
        return -EINVAL;
    }

    // Check if Bluetooth is ready
    if (!bt_is_ready()) {
        LOG_ERR("Bluetooth is not initialized");
        return -EAGAIN;
    }

    // Check name length against CONFIG_BT_DEVICE_NAME_MAX
    if (strlen(argv[1]) >= CONFIG_BT_DEVICE_NAME_MAX) {
        LOG_ERR("Name too long (max %d characters)", 
                CONFIG_BT_DEVICE_NAME_MAX - 1);
        return -EINVAL;
    }

    // Get our own address
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);

    if (count == 0) {
        LOG_ERR("Could not get device address");
        return -EIO;
    }

    // Update BT device name first
    int err = bt_set_name(argv[1]);
    if (err) {
        LOG_ERR("Failed to set BT name (err %d)", err);
        return err;
    }

    // Update advertising with new name
    err = update_advertising_data(argv[1]);
    if (err) {
        LOG_ERR("Failed to update advertising data (err %d)", err);
        return err;
    }

    // Save to settings
    err = settings_save_one("bt/device_name", argv[1], strlen(argv[1]));
	if (err) {
        LOG_ERR("Failed to save name to flash (err %d)", err);
		return err;
	}
	
    // Update controller name
    strncpy(controller.controllerName, argv[1], sizeof(controller.controllerName) - 1);
    controller.controllerName[sizeof(controller.controllerName) - 1] = '\0';

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
    LOG_INF("Device %s renamed to: %s", addr_str, argv[1]);

	return 0;
}
// Shell command version
static int cmd_get_address(const struct shell *shell, size_t argc, char **argv)
{
    bt_addr_le_t addr;
    size_t count = 1;
    
    bt_id_get(&addr, &count);
  
    if (count > 0) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
        shell_print(shell, "Device address: %s", addr_str);
    }
    return 0;
} 

static void list_bonds_cb(const struct bt_bond_info *info, void *user_data)
{
    const struct shell *shell = (const struct shell *)user_data;
    char addr_str[BT_ADDR_LE_STR_LEN];
    
    bt_addr_le_to_str(&info->addr, addr_str, sizeof(addr_str));
    shell_print(shell, "Bonded device: %s", addr_str);
}

static int cmd_list_bonds(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "Listing bonded devices...");
    bt_foreach_bond(BT_ID_DEFAULT, list_bonds_cb, (void *)shell);
    return 0;
}
static int cmd_send_info(const struct shell *shell, size_t argc, char **argv)
{
    if (!is_connected) {
          shell_error(shell, "No device connected");
        return -ENOTCONN;
    }

    LOG_INF("Manually triggering device info send");
    send_device_info(current_conn);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sensors,
    SHELL_CMD(list, NULL, "List all sensor instances", cmd_print_sensors),
    SHELL_CMD(rename, NULL, "Rename device <new_name>", cmd_rename_device),
    SHELL_CMD(address, NULL, "Get device address", cmd_get_address),
    SHELL_CMD(bonds, NULL, "List bonded devices", cmd_list_bonds),
    SHELL_CMD(info, NULL, "Send device info packet", cmd_send_info),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sensors, &sub_sensors, "Sensor commands", NULL);

// Data stream response packet format
#define STREAM_HEADER 0xAA
#define SENSOR_BLOCK_SIZE 8  // Size of each sensor data block

// Function to update sensor readings with simulated data
static void update_sensor_readings(void) {
    static uint32_t last_update = 0;
    uint32_t now = k_uptime_get_32();

    // Update each sensor based on its data rate
    for (int i = 0; i < controller.numberOfSensors; i++) {
        uint32_t update_interval = 1000 / sensor_instances[i].meta->dataRate;  // ms between updates
        
        if (now - last_update >= update_interval) {
            // Generate simulated data based on sensor type
            switch (sensor_instances[i].meta->sensorType) {
                case 1:  // Temperature (20-30°C)
                    sensor_instances[i].sensorReading = 20.0f + (rand() % 100) / 10.0f;
                    break;
                    
                case 2:  // Pressure (980-1020 hPa)
                    sensor_instances[i].sensorReading = 980.0f + (rand() % 400) / 10.0f;
                    break;
                    
                case 3:  // Force (0-100N)
                    sensor_instances[i].sensorReading = (rand() % 1000) / 10.0f;
                    break;
                    
                default:
                    // Generic sine wave for unknown sensors
                    sensor_instances[i].sensorReading = 50.0f + 
                        (sinf((float)now / 1000.0f) * 25.0f);
            }
            
            // Update timestamp
            sensor_instances[i].unixTime = now / 1000;
            
            LOG_DBG("Sensor %d updated: %.2f", 
                    sensor_instances[i].instanceId,
                    sensor_instances[i].sensorReading);
        }
    }
    
    last_update = now;
}

static void update_stream_data(uint8_t *payload, size_t max_size) {
    size_t offset = 0;

    // Header (1 byte)
    payload[offset++] = STREAM_HEADER;

    // Number of sensors (1 byte)
    payload[offset++] = controller.numberOfSensors;

    // Battery level (1 byte - convert float to uint8_t percentage)
    payload[offset++] = (uint8_t)controller.batteryLevel;

    // Status (1 byte)
    payload[offset++] = controller.status;

    // Unix timestamp (5 bytes)
    uint64_t timestamp = k_uptime_get() / 1000;  // Convert to seconds
    for (int i = 0; i < 5; i++) {
        payload[offset++] = (timestamp >> ((4-i) * 8)) & 0xFF;
    }

    // Add sensor data blocks
    for (int i = 0; i < controller.numberOfSensors; i++) {
        // Sensor ID (1 byte)
        payload[offset++] = sensor_instances[i].instanceId;

        // Array type (1 byte)
        payload[offset++] = 0x02;  // Example array type

        // Data type (1 byte)
        payload[offset++] = sensor_instances[i].meta->dataType;

        // Sensor data (4 bytes - float)
        uint32_t sensor_data;
        memcpy(&sensor_data, &sensor_instances[i].sensorReading, sizeof(float));
        payload[offset++] = (sensor_data >> 24) & 0xFF;
        payload[offset++] = (sensor_data >> 16) & 0xFF;
        payload[offset++] = (sensor_data >> 8) & 0xFF;
        payload[offset++] = sensor_data & 0xFF;

        // Sensor ID for next block (1 byte)
        payload[offset++] = sensor_instances[i].meta->sensorType;
    }

    LOG_DBG("Stream packet size: %d bytes", offset);
}

int main(void)
{
	int err;

	// Set thread priority to high for faster processing
	k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(0));

	name_threads();

	// Initialize settings subsystem first
	err = settings_subsys_init();
	if (err) {
		LOG_ERR("Failed to initialize settings subsystem (err %d)", err);
		return err;
	}

	// Load settings before BT init
	settings_load();

	// Initialize Bluetooth
	err = bluetooth_init();
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

	err = init_button();
	if (err) {
		LOG_ERR("Button init failed (err %d)", err);
		return 0;
	}

	LOG_INF("Starting sensor initialization...");

    // Create sensor instances with sequential indexing
	create_sensor_instance(0, 1, 2, 1, 1, 1, 100);  // Temperature sensor
	create_sensor_instance(1, 2, 1, 1, 1, 1, 50);   // Humidity sensor
	create_sensor_instance(2, 3, 2, 1, 1, 1, 25);   // Pressure sensor

    get_device_address();
	// Initialize controller information
	update_controller_info(
		controller.controllerName,  // Use the name from Kconfig
		controller.controllerAddress,    // Address will be updated when connected
		0x1010,                 // Controller type
		1.0f,                   // Firmware version
		100.0f,                 // Battery level (%)
		0x01                    // Status (OK)
	);

	// Now you can easily iterate through all sensors
	for (int i = 0; i < sensor_index; i++) {
		const SensorLookup* sensor_info = get_sensor_info(sensor_instances[i].meta->sensorType);
		LOG_INF("Sensor %d: Type %s reading: %.2f %s", 
				i, 
				sensor_info->name,
				sensor_instances[i].sensorReading,
				sensor_info->units);
	}

	while (1) {
		if (send_data_enabled && is_connected && notifications_enabled) {
			// Update sensor readings
			update_sensor_readings();

			// Calculate max size needed: header(1) + num_sensors(1) + battery(1) + status(1) + 
			// timestamp(5) + (sensor_blocks * 8)
			static uint8_t stream_payload[1 + 1 + 1 + 1 + 5 + (MAX_SENSORS * 8)];
			update_stream_data(stream_payload, sizeof(stream_payload));
			int result = send_data_via_bluetooth(stream_payload, sizeof(stream_payload));
			if (result != 0) {
				LOG_ERR("Failed to send data: %d", result);
				if (result == -ENOTCONN || result == -EAGAIN) {
					continue;
				}
			} else {
				packet_count++;
				
				// Print stats every second
				uint32_t now = k_uptime_get_32();
				if (now - last_print >= 1000) {
					LOG_INF("Packets sent in last second: %d", packet_count);
					packet_count = 0;
					last_print = now;
				}
				
				// Print connection info every 5 seconds
				if (now - last_conn_info >= 5000) {
					print_conn_info(current_conn);
					last_conn_info = now;
				}
				
				simulate_battery_drain();  // Check and update battery level
			}
		}
		k_sleep(K_MSEC(SEND_RATE_MS));  // Minimal sleep to allow other tasks to run
	}

	return 0;
}
