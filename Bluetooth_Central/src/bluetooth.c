#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/sys/byteorder.h>
#include <stdlib.h>
#include "bluetooth.h"
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/conn.h>
#include <dk_buttons_and_leds.h>


#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_DATA_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 data_char_uuid = BT_UUID_INIT_128(BT_UUID_DATA_CHAR_VAL);

LOG_MODULE_REGISTER(Bluetooth_Central, LOG_LEVEL_INF);

// Move these to the top with other static declarations
static K_SEM_DEFINE(bt_init_ok, 0, 1);

// Forward declare the bt_ready callback
static void bt_ready(int err);

// Forward declarations first
static uint8_t notify_func(struct bt_conn *conn,
                          struct bt_gatt_subscribe_params *params,
                          const void *data, uint16_t length);

static uint8_t discover_func(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           struct bt_gatt_discover_params *params);

static uint8_t discover_ccc_func(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           struct bt_gatt_discover_params *params);

// Then the static variables
static struct bt_conn *current_conn;
static struct bluetooth_callbacks bt_callbacks;
static bool is_scanning = false;
static char str[BT_UUID_STR_LEN];  // Buffer for UUID string conversion
bool notifications_enabled = false;

// GATT discovery variables - make these static and properly initialized
static struct bt_gatt_discover_params discover_params = {
    .func = discover_func,
    .type = BT_GATT_DISCOVER_PRIMARY,
    .uuid = &custom_service_uuid.uuid,
    .start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
    .end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
};
static struct bt_gatt_discover_params ccc_discover_params;

static struct bt_gatt_subscribe_params subscribe_params = {
    .notify = notify_func,
    .value = BT_GATT_CCC_NOTIFY,
    .ccc_handle = 0,
    .value_handle = 0,
};

// Device storage
static BTDevice_t found_devices[MAX_DEVICES];
static int num_found_devices = 0;


static bool security_params_requested = false;


static void list_bonds_callback(const struct bt_bond_info *info, void *user_data);
static int bond_count = 0;  // Add this with other static variables

// Add at the top with other structs
struct peer_name {
    bt_addr_le_t addr;
    char name[MAX_NAME_LENGTH];
    bool valid;
};

// Add with other static variables
static struct peer_name peer_names[CONFIG_BT_MAX_PAIRED];

// Forward declare the pairing complete callback
static void pairing_complete(struct bt_conn *conn, bool bonded);

// Update the auth callbacks
static struct bt_conn_auth_cb auth_cb = {
    .passkey_display = NULL,
    .passkey_entry = NULL,
    .passkey_confirm = NULL,
    .oob_data_request = NULL,
};

// Add security info callbacks
static struct bt_conn_auth_info_cb auth_info_cb = {
    .pairing_complete = pairing_complete,
};

// Unused variable - keeping for future reference
// static struct bt_conn_cb conn_callbacks;

// Add this at the top with other static variables
static struct bt_gatt_subscribe_params *current_subscribe_params = NULL;

// Add this callback function declaration at the top
static void write_callback(struct bt_conn *conn, uint8_t err, 
                         struct bt_gatt_write_params *params);

// Add the write params as a static variable to preserve it during async operation
static struct bt_gatt_write_params write_params;

// Add this at the top with other static variables
static bool is_enabling_notifications = false;  // Track whether we're enabling or disabling

// Add this declaration near the top with other forward declarations
static int peer_name_settings_set(const char *key, size_t len_rd,
                                settings_read_cb read_cb, void *cb_arg);

// Register our settings handler
SETTINGS_STATIC_HANDLER_DEFINE(bt_peers, "bt", NULL, peer_name_settings_set, NULL, NULL);

// Add this function before load_peer_name
static int settings_direct_loader(const char *key, size_t len,
                                settings_read_cb read_cb, void *cb_arg,
                                void *param)
{
    char *name = (char *)param;
    size_t size = MAX_NAME_LENGTH;  // Use the maximum size we support

    if (len > size - 1) {  // Leave room for null terminator
        return -ENOSPC;
    }

    ssize_t ret = read_cb(cb_arg, name, len);
    if (ret < 0) {
        return ret;
    }

    // Ensure null termination
    name[ret] = '\0';

    // Validate the loaded data
    if (ret > 0) {
        LOG_DBG("Loaded name data: len=%d, content='%s'", ret, name);
        // Check if the content is valid (not just whitespace)
        bool valid = false;
        for (int i = 0; i < ret; i++) {
            if (name[i] != ' ' && name[i] != '\n' && name[i] != '\r') {
                valid = true;
                break;
            }
        }
        if (!valid) {
            LOG_WRN("Loaded name contains only whitespace");
            return -EINVAL;
        }
    }

    return ret;
}

static int load_peer_name(const bt_addr_le_t *addr, char *name, size_t size)
{
    char key[32];
    char addr_str[BT_ADDR_LE_STR_LEN];
    
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    LOG_INF("Attempting to load name for device %s", addr_str);

    // Initialize name buffer
    memset(name, 0, size);

    // Use same key format as settings handler (just dev/ without bt/)
    snprintf(key, sizeof(key), "dev/%02x%02x%02x%02x%02x%02x",
             addr->a.val[5], addr->a.val[4], addr->a.val[3],
             addr->a.val[2], addr->a.val[1], addr->a.val[0]);

    LOG_INF("Using settings key: '%s'", key);
    int err = settings_load_subtree_direct(key, settings_direct_loader, name);
    if (err < 0) {
        LOG_ERR("Failed to load name (err %d)", err);
        return err;
    } else if (err == 0) {
        LOG_WRN("No data found for key");
        return -ENOENT;
    } else {
        LOG_INF("Successfully loaded name '%s' for peer", name);
        return 0;
    }
}

// Add this function to save a name
static int save_peer_name(const bt_addr_le_t *addr, const char *name)
{
    char key[40]; // "bt/dev/" + 12 hex bytes + '\0' => 19+ is safe

    // Use a key format: "bt/dev/xxxxxxxxxxxx"
    snprintf(key, sizeof(key), "bt/dev/%02x%02x%02x%02x%02x%02x",
             addr->a.val[5], addr->a.val[4], addr->a.val[3],
             addr->a.val[2], addr->a.val[1], addr->a.val[0]);

    LOG_DBG("Saving name '%s' with key '%s'", name, key);
    int err = settings_save_one(key, name, strlen(name));
    if (err) {
        LOG_ERR("Failed to save peer name (err %d)", err);
    } else {
        LOG_DBG("Successfully saved name '%s' for peer", name);
        
        // Update runtime array
        for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
            if (!peer_names[i].valid || 
                bt_addr_le_cmp(&peer_names[i].addr, addr) == 0) {
                bt_addr_le_copy(&peer_names[i].addr, addr);
                strncpy(peer_names[i].name, name, sizeof(peer_names[i].name) - 1);
                peer_names[i].name[sizeof(peer_names[i].name) - 1] = '\0';
                peer_names[i].valid = true;
                break;
            }
        }
    }
    return err;
}

// Add this function to handle settings load
static int peer_name_settings_set(const char *key, size_t len_rd,
                                settings_read_cb read_cb, void *cb_arg)
{
    const char *next;

    if (!key) {
        return -ENOENT;
    }

    // Check if this is a device key
    if (!settings_name_steq(key, "dev", &next) || !next) {
        return -ENOENT;
    }

    // Get the address from the key
    char addr_str[13];  // 12 chars for address + null terminator
    strncpy(addr_str, next, sizeof(addr_str) - 1);
    addr_str[sizeof(addr_str) - 1] = '\0';

    // Convert string address to bt_addr_le_t
    bt_addr_le_t addr = {
        .type = BT_ADDR_LE_RANDOM,
        .a.val = {0}
    };
    
    // Convert hex string to address bytes
    for (int i = 0; i < 6; i++) {
        char byte_str[3] = {addr_str[i*2], addr_str[i*2+1], '\0'};
        unsigned int byte_val;
        sscanf(byte_str, "%02x", &byte_val);
        addr.a.val[5-i] = (uint8_t)byte_val;
    }

    // Read the name from settings
    char name[MAX_NAME_LENGTH];
    ssize_t name_len = read_cb(cb_arg, name, sizeof(name) - 1);
    if (name_len < 0) {
        return name_len;
    }
    name[name_len] = '\0';

    LOG_INF("Loading name '%s' from settings for key %s", name, key);

    // Store in runtime array
    bool stored = false;
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        if (peer_names[i].valid) {
            if (bt_addr_le_cmp(&peer_names[i].addr, &addr) == 0) {
                // Update existing entry
                strncpy(peer_names[i].name, name, MAX_NAME_LENGTH - 1);
                peer_names[i].name[MAX_NAME_LENGTH - 1] = '\0';
                stored = true;
                LOG_INF("Updated name '%s' in runtime array[%d]", name, i);
                break;
            }
        } else if (!stored) {
            // Use first empty slot
            bt_addr_le_copy(&peer_names[i].addr, &addr);
            strncpy(peer_names[i].name, name, MAX_NAME_LENGTH - 1);
            peer_names[i].name[MAX_NAME_LENGTH - 1] = '\0';
            peer_names[i].valid = true;
            stored = true;
            LOG_INF("Stored name '%s' in runtime array[%d]", name, i);
            break;
        }
    }

    return 0;
}

// Use pairing_complete as a connection callback instead
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    if (!bonded) {
        return;
    }

    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    // Find device in scan results
    for (int i = 0; i < num_found_devices; i++) {
        if (bt_addr_le_cmp(addr, &found_devices[i].addr) == 0) {
            if (found_devices[i].name[0] != '\0') {
                save_peer_name(addr, found_devices[i].name);
                LOG_INF("Saved name '%s' for bonded device %s",
                        found_devices[i].name, addr_str);
            }
            break;
        }
    }
}

// Implement the callback
static void write_callback(struct bt_conn *conn, uint8_t err,
                         struct bt_gatt_write_params *params)
{
    if (err) {
        LOG_ERR("Write failed (err %d)", err);
        return;
    }

    if (is_enabling_notifications) {
        LOG_INF("Notifications enabled successfully callback");
        notifications_enabled = true;
        subscribe_params.notify = notify_func;  // Set the notify callback
    } else {
        LOG_INF("Notifications disabled successfully");
        notifications_enabled = false;
    }
}

// Manufacturer name lookup
static const char* get_manufacturer_name(uint16_t manufacturer_id) {
    switch (manufacturer_id) {
        // Major consumer electronics manufacturers
        case 0x004C: return "Apple";
        case 0x0075: return "Samsung";
        case 0x0059: return "Nordic Semiconductor"; 
        case 0x003D: return "Google";
        case 0x02E0: return "Xiaomi";
        case 0x00A0: return "Sony";
        case 0x05A7: return "Sonos";
        case 0x00C2: return "Intel";
        case 0x0030: return "ST Microelectronics";
        case 0x0499: return "Ruuvi Innovations";
        case 0x0001: return "Ericsson";
        case 0x0002: return "IBM";
        case 0x000D: return "Texas Instruments";
        case 0x0078: return "Laird";
        case 0x0077: return "Microchip";
        case 0x0006: return "Microsoft";
        case 0x02FF: return "Silicon Labs";
        case 0x0047: return "Garmin";
        case 0x038F: return "Xiaomi";
        
        // Add more manufacturers as needed
        default: return NULL;
    }
}

static void sort_devices_by_rssi(void) {
    for (int i = 0; i < num_found_devices - 1; i++) {
        for (int j = 0; j < num_found_devices - i - 1; j++) {
            if (found_devices[j].rssi < found_devices[j + 1].rssi) {
                BTDevice_t temp = found_devices[j];
                found_devices[j] = found_devices[j + 1];
                found_devices[j + 1] = temp;
            }
        }
    }
}

static bool data_cb(struct bt_data *data, void *user_data)
{
    BTDevice_t *device = (BTDevice_t *)user_data;
    
    LOG_DBG("Processing AD type: 0x%02X (len=%d)", data->type, data->data_len);
    
    switch (data->type) {
        case BT_DATA_NAME_SHORTENED:
        case BT_DATA_NAME_COMPLETE:
            if (data->data_len > 0) {
                uint8_t name_len = MIN(data->data_len, MAX_NAME_LENGTH - 1);
                memcpy(device->name, data->data, name_len);
                device->name[name_len] = '\0';
                LOG_DBG("Found device name: '%s' (len=%d, type=0x%02X)", 
                       device->name, data->data_len, data->type);
            }
            break;

        case BT_DATA_GAP_APPEARANCE:
            if (data->data_len == 2) {
                uint16_t appearance = sys_get_le16(data->data);
                LOG_DBG("Device appearance: 0x%04X", appearance);
            }
            break;

        case BT_DATA_FLAGS:
            if (data->data_len > 0) {
                uint8_t flags = data->data[0];
                device->connectable = (flags & BT_LE_AD_GENERAL) != 0;
                LOG_DBG("Device flags: 0x%02X (connectable: %d)", 
                       flags, device->connectable);
            }
            break;

        case BT_DATA_MANUFACTURER_DATA:
            if (data->data_len >= 2) {
                uint16_t manufacturer_id = sys_get_le16(data->data);
                LOG_DBG("Manufacturer Data: ID=0x%04X, len=%d", 
                       manufacturer_id, data->data_len);
                
                // Print raw manufacturer data for debugging
                LOG_DBG("Raw manufacturer data:");
                for (int i = 2; i < data->data_len; i++) {
                    LOG_DBG("  [%d]: 0x%02X (%c)", 
                           i-2, data->data[i],
                           (data->data[i] >= 32 && data->data[i] <= 126) ? 
                           data->data[i] : '.');
                }
                
                const char* mfg_name = get_manufacturer_name(manufacturer_id);
                
                // Store manufacturer name if available
                if (mfg_name) {
                    strncpy(device->manufacturer_name, mfg_name,
                           MAX_MANUFACTURER_NAME_LENGTH - 1);
                    device->manufacturer_name[MAX_MANUFACTURER_NAME_LENGTH - 1] = '\0';
                }

                // Only try to extract name if we don't already have one
                if (device->name[0] == '\0') {
                    // First try to find Complete Local Name in the manufacturer data
                    const uint8_t *data_ptr = data->data;
                    int remaining_len = data->data_len;
                    
                    while (remaining_len > 2) {
                        uint8_t field_len = data_ptr[0];
                        if (field_len == 0 || field_len > remaining_len) {
                            break;
                        }
                        
                        uint8_t field_type = data_ptr[1];
                        if (field_type == BT_DATA_NAME_COMPLETE || 
                            field_type == BT_DATA_NAME_SHORTENED) {
                            if (field_len > 1) {
                                uint8_t name_len = MIN(field_len - 1, MAX_NAME_LENGTH - 1);
                                memcpy(device->name, &data_ptr[2], name_len);
                                device->name[name_len] = '\0';
                                LOG_DBG("Found name in manufacturer data: '%s' (len=%d)", 
                                       device->name, name_len);
                                break;
                            }
                        }
                        
                        data_ptr += field_len + 1;
                        remaining_len -= field_len + 1;
                    }

                    // If no name found, try manufacturer-specific formats
                    if (device->name[0] == '\0') {
                        // ... rest of manufacturer-specific parsing ...
                    }
                }
            }
            break;

        case BT_DATA_SVC_DATA16:
            LOG_DBG("Service Data (len=%d):", data->data_len);
            for (int i = 0; i < data->data_len; i++) {
                LOG_DBG("[%d]: 0x%02X", i, data->data[i]);
            }
            break;

        case BT_DATA_UUID16_SOME:
        case BT_DATA_UUID16_ALL:
            LOG_DBG("16-bit UUIDs present");
            break;

        case BT_DATA_UUID32_SOME:
        case BT_DATA_UUID32_ALL:
            LOG_DBG("32-bit UUIDs present");
            break;

        case BT_DATA_UUID128_SOME:
        case BT_DATA_UUID128_ALL:
            LOG_DBG("128-bit UUIDs present");
            break;

        case BT_DATA_TX_POWER:
            if (data->data_len == 1) {
                device->tx_power = (int8_t)data->data[0];
                LOG_DBG("TX Power: %d dBm", device->tx_power);
            }
            break;

        default:
            LOG_DBG("Unhandled AD type: 0x%02X (len=%d)", data->type, data->data_len);
            break;
    }
    return true;
}


static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                        struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    
    // Check if device already exists
    for (int i = 0; i < num_found_devices; i++) {
        if (bt_addr_le_cmp(addr, &found_devices[i].addr) == 0) {
            // Update RSSI if it changed significantly
            if (abs(found_devices[i].rssi - rssi) > 5) {
                found_devices[i].rssi = rssi;
            }
            // Always parse advertising data to catch name updates
            if (ad) {
                bt_data_parse(ad, data_cb, &found_devices[i]);
            }
            return;
        }
    }

    if (num_found_devices >= MAX_DEVICES) {
        return;
    }

    // Store new device
    bt_addr_le_copy(&found_devices[num_found_devices].addr, addr);
    found_devices[num_found_devices].rssi = rssi;
    found_devices[num_found_devices].connected = false;
    found_devices[num_found_devices].name[0] = '\0';

    // Parse advertising data
    if (ad) {
        bt_data_parse(ad, data_cb, &found_devices[num_found_devices]);
    }

    num_found_devices++;
    sort_devices_by_rssi();
}

// Add these globals to store device info
SensorInstance sensor_instances[MAX_SENSORS];
BluetoothController received_controller = {0};
static SensorMetaData received_sensor_metadata[MAX_SENSORS];
static uint8_t num_sensor_instances = 0;
static bool device_info_received = false;

static uint8_t notify_func(struct bt_conn *conn,
                          struct bt_gatt_subscribe_params *params,
                          const void *data, uint16_t length)
{
    static uint32_t stream_packet_count = 0;
    static uint32_t other_packet_count = 0;
    static int64_t last_print_time = 0;
    
    // Skip if this is a CCC write confirmation (data will be NULL)
    if (!data) {
        LOG_DBG("CCC write confirmation received");
        return BT_GATT_ITER_CONTINUE;
    }
    
    const uint8_t *bytes = (const uint8_t *)data;
    uint8_t header = bytes[0];
    
    // Count packets based on type
    if (header == DATA_STREAM_RESPONSE) {
        stream_packet_count++;
    } else {
        other_packet_count++;
    }
    
    int64_t now = k_uptime_get();
    if (now - last_print_time >= 1000) {
        LOG_INF("Last second - Stream packets: %u, Other packets: %u", 
                stream_packet_count, other_packet_count);
        stream_packet_count = 0;
        other_packet_count = 0;
        last_print_time = now;
    }

    // Process the notification data
    switch (header) {
        case PACKET_HEADER_DEVICE_INFO: {
            if (length < (1 + sizeof(BluetoothController))) {
                LOG_ERR("Device info packet too short");
                return BT_GATT_ITER_CONTINUE;
            }

            // Get controller info
            size_t offset = 1;  // Skip header
            memcpy(&received_controller, &bytes[offset], sizeof(BluetoothController));
            offset += sizeof(BluetoothController);

            if (received_controller.numberOfSensors > MAX_SENSORS) {
                LOG_WRN("Device reports more sensors than we can handle: %d", 
                        received_controller.numberOfSensors);
                received_controller.numberOfSensors = MAX_SENSORS;
            }

            // Get sensor metadata and create instances
            for (int i = 0; i < received_controller.numberOfSensors; i++) {
                // Store sensor metadata
                memcpy(&received_sensor_metadata[num_sensor_instances], 
                       &bytes[offset], sizeof(SensorMetaData));
                offset += sizeof(SensorMetaData);

                // Create sensor instance
                sensor_instances[num_sensor_instances].instanceId = num_sensor_instances + 1;
                sensor_instances[num_sensor_instances].peripheralInstanceID = i + 1;
                sensor_instances[num_sensor_instances].sensorConnected = 1;
                sensor_instances[num_sensor_instances].meta = &received_sensor_metadata[num_sensor_instances];
                sensor_instances[num_sensor_instances].controller = &received_controller;
                sensor_instances[num_sensor_instances].sensorReading = 0.0f;
                sensor_instances[num_sensor_instances].unixTime = k_uptime_get() / 1000;

                num_sensor_instances++;
            }

            device_info_received = true;

            // Log the received information
            LOG_INF("Received device information:");
            LOG_INF("  Controller: %s", received_controller.controllerName);
            LOG_INF("  Address: %s", received_controller.controllerAddress);
            LOG_INF("  Type: 0x%04X", received_controller.controllerType);
            LOG_INF("  Firmware: %.2f", (double)received_controller.firmwareVersion);
            LOG_INF("  Battery: %.1f%%", (double)received_controller.batteryLevel);
            LOG_INF("  Status: 0x%02X", received_controller.status);
            LOG_INF("  Number of sensors: %d", received_controller.numberOfSensors);
            
            // Log sensor instances
            for (int i = 0; i < received_controller.numberOfSensors; i++) {
                LOG_INF("\nSensor Instance %d:", sensor_instances[i].instanceId);
                LOG_INF("  Peripheral Instance ID: %d", sensor_instances[i].peripheralInstanceID);
                LOG_INF("  Connected: %s", sensor_instances[i].sensorConnected ? "Yes" : "No");
                LOG_INF("  Sensor Type: 0x%04X", sensor_instances[i].meta->sensorType);
                LOG_INF("  Reading Size: %d", sensor_instances[i].meta->readingSize);
                LOG_INF("  Data Rate: %d Hz", sensor_instances[i].meta->dataRate);
                LOG_INF("  Last Reading: %.2f", (double)sensor_instances[i].sensorReading);
                LOG_INF("  Last Update: %lld", sensor_instances[i].unixTime);
            }
            break;
        }

        case DATA_STREAM_RESPONSE:
        
            if (length >= 9) {  // Minimum length: header(1) + num_sensors(1) + battery(1) + status(1) + timestamp(5)
                uint8_t num_sensors = bytes[1];
                uint8_t battery_level = bytes[2];
                uint8_t status = bytes[3];
                
                // Extract timestamp (5 bytes)
                uint64_t timestamp = 0;
                for (int i = 0; i < 5; i++) {
                    timestamp |= ((uint64_t)bytes[4 + i] << ((4-i) * 8));
                }

                // Update controller info
                received_controller.batteryLevel = (float)battery_level;
                received_controller.status = status;

                size_t idx = 9;  // Start of sensor data blocks
                
                // Process each sensor data block
                for (int i = 0; i < num_sensors && idx < length; i++) {
                    if (idx + 8 > length) {  // Ensure we have enough data for one block
                        LOG_WRN("Incomplete sensor data block");
                        break;
                    }

                    uint8_t sensor_id = bytes[idx++];
                    idx++; // Skip array_type byte
                    uint8_t data_type = bytes[idx++];
                    
                    // Read 4 bytes of sensor data
                    uint32_t sensor_data = 0;
                    for (int j = 0; j < 4; j++) {
                        sensor_data = (sensor_data << 8) | bytes[idx++];
                    }

                    uint8_t sensor_type = bytes[idx++];
                    
                    // Find matching sensor instance
                    for (int j = 0; j < received_controller.numberOfSensors; j++) {
                        if (sensor_instances[j].instanceId == sensor_id) {
                            // Update sensor type if needed
                            sensor_instances[j].meta->sensorType = sensor_type;
                            
                            // Convert sensor data based on data type
                            switch (data_type) {
                                case 0x01: // INT8
                                    sensor_instances[j].sensorReading = (int8_t)sensor_data;
                                    break;
                                case 0x02: // UINT8
                                    sensor_instances[j].sensorReading = (uint8_t)sensor_data;
                                    break;
                                case 0x03: // INT16
                                    sensor_instances[j].sensorReading = (int16_t)sensor_data;
                                    break;
                                case 0x04: // UINT16
                                    sensor_instances[j].sensorReading = (uint16_t)sensor_data;
                                    break;
                                case 0x07: // FLOAT
                                    float value;
                                    memcpy(&value, &sensor_data, sizeof(float));
                                    sensor_instances[j].sensorReading = value;
                                    break;
                                default:
                                    LOG_WRN("Unsupported data type: 0x%02X", data_type);
                                    break;
                            }
                            
                            sensor_instances[j].unixTime = timestamp;
                            LOG_DBG("Sensor %d updated: value=%.2f, type=0x%02X, time=%lld", 
                                   sensor_id,
                                   (double)sensor_instances[j].sensorReading,
                                   sensor_type,
                                   sensor_instances[j].unixTime);
                            break;
                        }
                    }
                }

                // Print sensor readings every 5 seconds
                static uint32_t last_print = 0;
                uint32_t now = k_uptime_get() / 1000;
                if (now - last_print >= 5) {
                    for (int i = 0; i < num_sensor_instances; i++) {
                        LOG_INF("Sensor %d reading: %.2f, time: %lld", 
                               sensor_instances[i].instanceId,
                               (double)sensor_instances[i].sensorReading,
                               sensor_instances[i].unixTime);
                    }
                    last_print = now;
                }
            }
            break;  

        // Handle other packet types...
        default:
            LOG_WRN("Unknown packet header: 0x%02X", header);
            break;
    }

    // Call callback if it exists
    if (bt_callbacks.data_received) {
        bt_callbacks.data_received(data, length);
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr,
                           struct bt_gatt_discover_params *params)
{
    int err;

    if (!attr) {
        LOG_INF("Discovery phase complete");
        if (params->type == BT_GATT_DISCOVER_PRIMARY) {
            // After discovering all services, start discovering characteristics
            params->type = BT_GATT_DISCOVER_CHARACTERISTIC;
            params->uuid = &data_char_uuid.uuid;
            params->start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
            params->end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;

            err = bt_gatt_discover(conn, params);
            if (err) {
                LOG_ERR("Characteristic discover failed (err %d)", err);
            }
            return BT_GATT_ITER_STOP;
        }
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
        
        // If this is our target characteristic
        if (bt_uuid_cmp(chrc->uuid, &data_char_uuid.uuid) == 0) {
            LOG_INF("Found target characteristic - handle: %u, value handle: %u", 
                    attr->handle, chrc->value_handle);
            subscribe_params.value_handle = chrc->value_handle;
            
            // Start separate CCC discovery
            memset(&ccc_discover_params, 0, sizeof(ccc_discover_params));
            static struct bt_uuid_16 ccc_uuid = BT_UUID_INIT_16(0x2902);
            ccc_discover_params.uuid = &ccc_uuid.uuid;
            ccc_discover_params.start_handle = chrc->value_handle + 1;
            ccc_discover_params.end_handle = chrc->value_handle + 10;
            ccc_discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
            ccc_discover_params.func = discover_ccc_func;
            
            LOG_INF("Starting CCC discovery from handle %u to %u (looking for UUID 0x2902)",
                    ccc_discover_params.start_handle,
                    ccc_discover_params.end_handle);
            
            err = bt_gatt_discover(conn, &ccc_discover_params);
            if (err) {
                LOG_ERR("Failed to start CCC discovery (err %d)", err);
            }
            return BT_GATT_ITER_STOP;
        }
        return BT_GATT_ITER_CONTINUE;
    }

    return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_ccc_func(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                struct bt_gatt_discover_params *params)
{
    if (!attr) {
        if (subscribe_params.ccc_handle == 0) {
            LOG_ERR("CCC descriptor not found");
            return BT_GATT_ITER_STOP;
        }
        LOG_INF("CCC Discovery completed successfully");
        // Enable notifications after discovery completes
        int err = enable_notifications();
    if (err) {
            if (err == -EAGAIN) {
                LOG_INF("Security not established yet, will retry after security is established");
            } else {
                LOG_ERR("Failed to enable notifications (err %d)", err);
            }
        } else {
            LOG_INF("Notifications enabled successfully");
        }
        return BT_GATT_ITER_STOP;
    }

    bt_uuid_to_str(attr->uuid, str, sizeof(str));
    LOG_INF("Found descriptor at handle %u with UUID %s", attr->handle, str);
    static struct bt_uuid_16 ccc_uuid = BT_UUID_INIT_16(0x2902);
    if (bt_uuid_cmp(attr->uuid, &ccc_uuid.uuid) == 0) {
        LOG_INF("Found CCC descriptor");
        subscribe_params.ccc_handle = attr->handle;
        return BT_GATT_ITER_CONTINUE;  // Continue to find all descriptors
    }

    return BT_GATT_ITER_CONTINUE;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %d)", err);
        return;
    }
    
    struct bt_conn_info info;
    err = bt_conn_get_info(conn, &info);
    if (!err) {
        LOG_INF("Connected with supervision timeout: %d ms", 
                info.le.timeout * 10);  // Timeout is in units of 10ms
    }

    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);
    
        // Notify callback first
    if (bt_callbacks.connected) {
        bt_callbacks.connected();
    }
    
    // Reset handles on new connection
    subscribe_params.value_handle = 0;
    subscribe_params.ccc_handle = 0;
    notifications_enabled = false;  // Reset notification state

    // Set security level first and WAIT for security_changed callback
    err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (err) {
        LOG_ERR("Failed to set security (err %d)", err);
    }

    // Don't start any discovery or parameter updates yet
    // Wait for security_changed callback
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    const bt_addr_le_t *remote_addr = bt_conn_get_dst(conn);
    bt_addr_le_to_str(remote_addr, addr, sizeof(addr));

    // Log detailed disconnect information
    LOG_INF("Disconnect callback triggered for %s", addr);
    LOG_INF("  Reason code: 0x%02x", reason);
    LOG_INF("  Connection pointer: %p", (void*)conn);
    LOG_INF("  Connection state: %s", current_conn ? "Connected" : "Disconnected");
    LOG_INF("  Notifications state: %d", notifications_enabled);

    // Log the specific disconnect reason
    switch (reason) {
        case BT_HCI_ERR_CONN_TIMEOUT:
            LOG_INF("Disconnected: SUPERVISION TIMEOUT from %s", addr);
            break;
        case BT_HCI_ERR_REMOTE_USER_TERM_CONN:
            LOG_INF("Disconnected: REMOTE USER TERMINATED from %s", addr);
            break;
        case BT_HCI_ERR_LOCALHOST_TERM_CONN:
            LOG_INF("Disconnected: LOCAL HOST TERMINATED from %s", addr);
            break;
        case BT_HCI_ERR_LL_RESP_TIMEOUT:
            LOG_INF("Disconnected: LMP RESPONSE TIMEOUT from %s", addr);
            break;
        default:
            LOG_INF("Disconnected: (reason 0x%02x) from %s", reason, addr);
            break;
    }

    // Manually disable notifications if enabled
    if (notifications_enabled) {
        LOG_INF("Attempting to disable notifications during disconnect...");
        disable_notifications();
        notifications_enabled = false;
        current_subscribe_params = NULL;
        LOG_INF("Notifications disabled during disconnect");
    }

    // Check for any lingering connections to this address
    struct bt_conn *lingering = bt_conn_lookup_addr_le(BT_ID_DEFAULT, remote_addr);
    if (lingering) {
        LOG_WRN("Found lingering connection to address %s", addr);
        
        // Force cleanup of the lingering connection
        if (lingering == current_conn) {
            LOG_DBG("Lingering connection matches current_conn");
            bt_conn_unref(current_conn);
            current_conn = NULL;
        }
        
        // Unref from the lookup and force an additional unref
        bt_conn_unref(lingering);
        bt_conn_unref(lingering);  // Force an additional cleanup
    }

    // Also cleanup the original connection if it wasn't the lingering one
    if (conn != lingering && current_conn == conn) {
        LOG_DBG("Cleaning up original connection");
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    // Reset all connection state
    subscribe_params.value_handle = 0;
    subscribe_params.ccc_handle = 0;
    security_params_requested = false;

    if (bt_callbacks.disconnected) {
        bt_callbacks.disconnected();
    }
}

static void le_phy_updated(struct bt_conn *conn,
                          struct bt_conn_le_phy_info *param)
{
    LOG_INF("PHY updated: TX PHY %s, RX PHY %s", 
            param->tx_phy == BT_GAP_LE_PHY_2M ? "2M" : "1M",
            param->rx_phy == BT_GAP_LE_PHY_2M ? "2M" : "1M");

    // Now start service discovery
    discover_params.type = BT_GATT_DISCOVER_PRIMARY;
    discover_params.uuid = &custom_service_uuid.uuid;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;

    int err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("Service discovery failed (err %d)", err);
    }
}

static bool le_param_req(struct bt_conn *conn,
                        struct bt_le_conn_param *param)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    
    LOG_INF("Central received parameter request from %s: interval (%.2f,%.2f) ms, latency %d, timeout %d ms",
            addr,
            (double)param->interval_min * 1.25,
            (double)param->interval_max * 1.25,
            param->latency,
            param->timeout * 10);

    // Reject any requests that don't match our desired parameters
    if (param->interval_min != 6 || param->interval_max != 6) {
        LOG_WRN("Rejecting connection parameters that don't match our requirements");
        return false;  // Reject the parameters
    }
    return true;  // Accept the parameters
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
                           uint16_t latency, uint16_t timeout)
{
    LOG_INF("Connection parameters updated: interval %.2f ms, latency %d, timeout %d ms",
            (double)interval * 1.25, latency, timeout * 10);

    // Now update PHY to 2M
    struct bt_conn_le_phy_param phy_param = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };

    int err = bt_conn_le_phy_update(conn, &phy_param);
    if (err && err != -EALREADY) {
        LOG_ERR("PHY update failed (err %d)", err);
    }
    // Service discovery will be started after PHY is updated
}

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Central MTU Updated for %s: TX: %d RX: %d bytes", addr, tx, rx);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
                           enum bt_security_err err)
{
    if (err) {
        LOG_ERR("Security failed: level %d err %d", level, err);
        return;
    }

    LOG_INF("Security changed: level %d", level);
    
    // Get current security info
    struct bt_conn_info info;
    int status = bt_conn_get_info(conn, &info);
    if (status) {
        LOG_ERR("Could not get connection info (err %d)", status);
        return;
    }

    // Only proceed if both sides have confirmed security level 2
    if (level >= BT_SECURITY_L2 && info.security.level >= BT_SECURITY_L2) {
        // Now it's safe to update connection parameters
        struct bt_le_conn_param conn_param = {
            .interval_min = 6,   // 30ms
            .interval_max = 6,   // 50ms
            .latency = 1,        // Allow up to 4 connection intervals to be skipped
            .timeout = 50,       // 4 second supervision timeout
        };
        
        status = bt_conn_le_param_update(conn, &conn_param);
        if (status && status != -EALREADY) {
            LOG_ERR("Connection parameter update failed (err %d)", status);
        }
        
        // Store the result of enable_notifications
       // status = enable_notifications();
       // if (status) {
       //     LOG_ERR("Failed to enable notifications (err %d)", status);
       // }
    }
}

// Update connection callbacks registration
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .le_phy_updated = le_phy_updated,
    .le_param_updated = le_param_updated,
    .le_param_req = le_param_req,
    .security_changed = security_changed,
};

// Register GATT callbacks
static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated
};

int start_scan(void)
{
    LOG_DBG("Starting scan procedure...");
    
    // Add timeout for Bluetooth readiness check
    int timeout = 100;
    LOG_DBG("Checking Bluetooth readiness...");
    while (!bt_is_ready() && timeout > 0) {
        LOG_DBG("Bluetooth not ready, retrying... (timeout: %d)", timeout);
        k_sleep(K_MSEC(10));
        timeout--;
    }

    if (!bt_is_ready()) {
        LOG_ERR("Bluetooth not ready after %d retries", 100 - timeout);
        return -EAGAIN;
    }
    LOG_DBG("Bluetooth is ready, proceeding with scan");

    int err;

    // Clear previous results
    num_found_devices = 0;
    memset(found_devices, 0, sizeof(found_devices));

    // Stop any ongoing scanning first
    LOG_DBG("Stopping any ongoing scan...");
    err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        LOG_ERR("Could not stop previous scanning (err %d)", err);
    }

    // Use predefined scan parameters
    const struct bt_le_scan_param *scan_param = BT_LE_SCAN_ACTIVE;

    LOG_INF("Starting active scan... (interval: 0x%04x, window: 0x%04x)",
            scan_param->interval, scan_param->window);

    err = bt_le_scan_start(scan_param, device_found);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d, bt_is_ready: %d)", 
                err, bt_is_ready());
        return err;
    }

    LOG_DBG("Scan started successfully");
    is_scanning = true;
    return 0;
}

static void force_hci_disconnect(struct bt_conn *conn)
{
    int err;

    if (!conn) {
        LOG_WRN("No active connection for HCI disconnect!");
        return;
    }

    // Get connection info
    struct bt_conn_info info;
    err = bt_conn_get_info(conn, &info);
    if (err) {
        LOG_ERR("Failed to get connection info (err %d)", err);
        return;
    }

    // Try more aggressive disconnect
    err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_POWER_OFF);
    if (err) {
        LOG_ERR("Failed to send aggressive disconnect (err %d)", err);
    } else {
        LOG_INF("Aggressive disconnect command sent successfully");
    }

    // Force immediate cleanup if disconnect failed
    if (err) {
        LOG_WRN("Disconnect failed, forcing immediate cleanup");
        bt_conn_unref(conn);
    }
}

void disconnect_current_connection(void)
{
    if (!current_conn) {
        LOG_WRN("No active connection to disconnect");
        return;
    }

    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(current_conn), addr, sizeof(addr));
    LOG_INF("Initiating disconnect from %s", addr);

    LOG_INF("Pre-disconnect state:");
    LOG_INF("  Connection state: %s", current_conn ? "Connected" : "Disconnected");
    LOG_INF("  Notifications enabled: %d", notifications_enabled);

    // Store connection reference for cleanup
    struct bt_conn *conn_to_cleanup = current_conn;
    bt_conn_ref(conn_to_cleanup);  // Add reference for our use
    bool cleanup_needed = true;

    // First unsubscribe if notifications are enabled
    if (notifications_enabled) {
        LOG_INF("Attempting to unsubscribe from notifications...");
        int err = bt_gatt_unsubscribe(current_conn, &subscribe_params);
    if (err) {
            LOG_WRN("Failed to unsubscribe from notifications (err %d)", err);
        } else {
            LOG_INF("Successfully unsubscribed from notifications");
        }
    }

    // Reset connection state immediately
    notifications_enabled = false;
    subscribe_params.value_handle = 0;
    subscribe_params.ccc_handle = 0;

    // Try graceful disconnect first
    int err = bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    
    if (err) {
        LOG_WRN("Graceful disconnect returned %d, trying force disconnect", err);
    } else {
        LOG_INF("Disconnect request sent, waiting briefly for completion...");
        k_sleep(K_MSEC(200));
    }

    // Force cleanup if we still have a connection
    if (cleanup_needed && current_conn == conn_to_cleanup) {
        LOG_INF("Connection cleanup needed, forcing disconnect");
        
        // Try forceful disconnect methods
        LOG_INF("Attempting HCI level disconnect...");
        force_hci_disconnect(current_conn);
        k_sleep(K_MSEC(200));  // Give more time for HCI disconnect
        
        // If still connected, try host level force disconnect
        if (current_conn == conn_to_cleanup) {
            LOG_WRN("HCI disconnect didn't work, trying host level force disconnect");
            err = bt_conn_disconnect(current_conn, BT_HCI_ERR_LOCALHOST_TERM_CONN);
            if (err) {
                LOG_ERR("Force disconnect failed (err %d)", err);
            }
            k_sleep(K_MSEC(100));  // Wait a bit more
        }

        // Force connection cleanup
        if (current_conn) {
            LOG_WRN("Connection still exists, forcing cleanup");
            struct bt_conn *old_conn = current_conn;
            current_conn = NULL;  // Clear current_conn first
            bt_conn_unref(old_conn);  // Release our reference
        }
        
        LOG_INF("Forced cleanup completed");
    }

    // Release our cleanup reference
    bt_conn_unref(conn_to_cleanup);
}

// Move these function definitions before bluetooth_init
void cleanup_old_settings(void)
{
    // Delete old format settings
    settings_delete("bt/peers");  // Delete the entire old peers subtree
    LOG_INF("Cleaned up old settings format");
}

void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_DBG("Bluetooth ready callback triggered");
    k_sem_give(&bt_init_ok);
}

void list_bonds_callback(const struct bt_bond_info *info, void *user_data)
{
    bt_addr_le_t *addrs = user_data;
    if (bond_count < CONFIG_BT_MAX_PAIRED) {
        bt_addr_le_copy(&addrs[bond_count], &info->addr);
        bond_count++;
    }
}

// Update bluetooth_init to call it
int bluetooth_init(struct bluetooth_callbacks *callbacks)
{
    int err;
    LOG_INF("Starting Bluetooth initialization...");

    // Store callbacks first
    bt_callbacks = *callbacks;

    // Initialize Bluetooth subsystem first
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    // Wait for initialization to complete
    k_sem_take(&bt_init_ok, K_FOREVER);

    // Now initialize settings after Bluetooth is ready
    err = settings_subsys_init();
    if (err) {
        LOG_ERR("Failed to init settings subsystem (err %d)", err);
        return err;
    }

    // Initialize peer names array
    memset(peer_names, 0, sizeof(peer_names));
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        peer_names[i].valid = false;
    }

    // Register security callbacks
    err = bt_conn_auth_cb_register(&auth_cb);
    if (err) {
        LOG_ERR("Failed to register auth callbacks (err %d)", err);
        return err;
    }

    // Register security info callbacks
    err = bt_conn_auth_info_cb_register(&auth_info_cb);
    if (err) {
        LOG_ERR("Failed to register auth info callbacks (err %d)", err);
        return err;
    }

    // Load settings after everything else is initialized
    err = settings_load();
    if (err) {
        LOG_ERR("Failed to load settings (err %d)", err);
        return err;
    }

    // Register GATT callbacks
    bt_gatt_cb_register(&gatt_callbacks);

    // Debug print loaded peer names
    LOG_INF("Loaded peer names:");
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        if (peer_names[i].valid) {
            char addr[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&peer_names[i].addr, addr, sizeof(addr));
            LOG_INF("[%d] Address: %s, Name: %s", i, addr, peer_names[i].name);
        }
    }

    return 0;
}

struct bt_conn *get_current_conn(void)
{
    return current_conn;
}

BTDevice_t* get_found_devices(void)
{
    return found_devices;
}

int get_num_found_devices(void)
{
    return num_found_devices;
}

void print_found_devices(void)
{
    LOG_INF("Found devices (%d):", num_found_devices);
    for (int i = 0; i < num_found_devices; i++) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&found_devices[i].addr, addr_str, sizeof(addr_str));
        
        LOG_INF("%d: [%s] Address: %s, Manufacturer: [%s], RSSI: %d",
                i + 1,  // +1 for human-readable numbering
                found_devices[i].name[0] ? found_devices[i].name : "(unknown)",
                addr_str,
                found_devices[i].manufacturer_name[0] ? 
                    found_devices[i].manufacturer_name : "unknown mfg",
                found_devices[i].rssi);
    }
}

int connect_to_device(int device_number)
{
    if (device_number <= 0 || device_number > num_found_devices) {
        LOG_ERR("Invalid device number %d", device_number);
        return -EINVAL;
    }

    // Check for lingering connections first
    if (current_conn) {
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(bt_conn_get_dst(current_conn), addr, sizeof(addr));
        LOG_WRN("Found lingering connection to %s, cleaning up", addr);
        bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(current_conn);
        current_conn = NULL;
        k_sleep(K_MSEC(500));  // Wait for cleanup
    }

    int idx = device_number - 1;
    BTDevice_t *device = &found_devices[idx];

    // Log the connection attempt details
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&device->addr, addr, sizeof(addr));
    LOG_INF("Attempting to connect to device %d: [%s] at address %s", 
            device_number,
            device->name[0] ? device->name : "(unknown)",
            addr);

    // Check if device is connectable
    if (!device->connectable) {
        LOG_ERR("Device %d is not connectable", device_number);
        return -EINVAL;
    }


    // Create connection with specific parameters
    struct bt_conn_le_create_param create_param = {
        .options = BT_CONN_LE_OPT_NONE,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window = BT_GAP_SCAN_FAST_WINDOW,
        .interval_coded = 0,
        .window_coded = 0,
        .timeout = 0,
    };

    // Use connection parameters with longer supervision timeout
    struct bt_le_conn_param conn_param = {
        .interval_min = 6,   // 30ms
        .interval_max = 6,   // 50ms
        .latency = 1,        // Allow up to 4 connection intervals to be skipped
        .timeout = 50,       // 4 second supervision timeout
    };

    int err = bt_conn_le_create(&device->addr, 
                                &create_param,
                                &conn_param,
                                &current_conn);

    if (err) {
        LOG_ERR("Create connection failed (err %d): %s", 
                err, 
                err == -EINVAL ? "Invalid parameters" :
                err == -ENOMEM ? "No memory" :
                err == -EBUSY ? "Busy" : "Unknown error");
        return err;
    }

    LOG_INF("Connecting to device %d: [%s]...", 
            device_number,
            device->name[0] ? device->name : "(unknown)");
    return 0;
}

int enable_notifications(void)
{
    if (!current_conn) {
        LOG_ERR("No active connection");
        return -EINVAL;
    }

    if (notifications_enabled) {
        LOG_WRN("Notifications already enabled");
        return 0;
    }

    if (!subscribe_params.value_handle || !subscribe_params.ccc_handle) {
        LOG_ERR("No valid handles for notifications");
        return -EINVAL;
    }

    LOG_INF("Enabling notifications (value handle: %d, ccc handle: %d)",
            subscribe_params.value_handle, subscribe_params.ccc_handle);

    // Set up subscription parameters
    subscribe_params.notify = notify_func;
    subscribe_params.value = BT_GATT_CCC_NOTIFY;

    // Use write_params and write_callback for subscription
    is_enabling_notifications = true;
    write_params.func = write_callback;
    write_params.handle = subscribe_params.ccc_handle;

    // Subscribe to notifications
    int err = bt_gatt_subscribe(current_conn, &subscribe_params);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to subscribe to notifications (err %d)", err);
        return err;
    }

    LOG_INF("Successfully subscribed to notifications");
        notifications_enabled = true;
    current_subscribe_params = &subscribe_params;

    // Get the connected device name
    const bt_addr_le_t *addr = bt_conn_get_dst(current_conn);
    if (!addr) {
        LOG_ERR("Could not get connection address");
        return -EINVAL;
    }

    // Find device name in peer_names array
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        if (peer_names[i].valid && 
            bt_addr_le_cmp(&peer_names[i].addr, addr) == 0) {
            
            // Send sensor info request automatically
            err = request_sensor_info(peer_names[i].name);
            if (err) {
                LOG_ERR("Failed to request sensor info (err %d)", err);
    return err;
            }
            LOG_INF("Automatically sent sensor info request");
            break;
        }
    }
    

    return 0;
}

int disable_notifications(void)
{
    if (!current_conn) {
        LOG_ERR("No active connection");
        return -ENOTCONN;
    }

    if (!notifications_enabled) {
        LOG_WRN("Notifications already disabled");
        return 0;
    }

    // Use write_params and write_callback for unsubscription
    is_enabling_notifications = false;
    write_params.func = write_callback;
    write_params.handle = subscribe_params.ccc_handle;

    int err = bt_gatt_unsubscribe(current_conn, &subscribe_params);
    if (err) {
        LOG_ERR("Failed to unsubscribe from notifications (err %d)", err);
        return err;
    }

    LOG_INF("Successfully unsubscribed from notifications");
        notifications_enabled = false;
    current_subscribe_params = NULL;
    return 0;
}

int bluetooth_list_bonds(void)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_t addrs[CONFIG_BT_MAX_PAIRED];
    
    bond_count = 0;
    bt_foreach_bond(BT_ID_DEFAULT, list_bonds_callback, addrs);

    if (bond_count == 0) {
        LOG_INF("No bonded devices found");
        return 0;
    }

    LOG_INF("Bonded devices:");
    for (int i = 0; i < bond_count; i++) {
        bt_addr_le_to_str(&addrs[i], addr, sizeof(addr));
        
        // Look up the name in our runtime array
        bool found = false;
        for (int j = 0; j < CONFIG_BT_MAX_PAIRED; j++) {
            if (peer_names[j].valid && 
                bt_addr_le_cmp(&peer_names[j].addr, &addrs[i]) == 0) {
                LOG_INF("Device %d: name='%s', address='%s'", 
                        i, peer_names[j].name, addr);
                found = true;
                break;
            }
        }
        
        if (!found) {
            LOG_WRN("No name found for device %d: address='%s'", i, addr);
        }
    }

    return 0;
}

int send_rename_command(const char *target_device_name, const char *new_name) {
    if (!target_device_name || !new_name) {
        LOG_ERR("Invalid parameters");
        return -EINVAL;
    }

    if (!current_conn) {
        LOG_ERR("No active connection");
        return -ENOTCONN;
    }

    if (!notifications_enabled || !subscribe_params.value_handle) {
        LOG_ERR("Notifications not enabled or characteristic not discovered");
        return -EINVAL;
    }

    size_t name_len = strlen(new_name);
    if (name_len == 0 || name_len > MAX_DEVICE_NAME_LENGTH) {
        LOG_ERR("Invalid name length: %d", name_len);
        return -EINVAL;
    }

    // Calculate total packet size: Header(1) + Timestamp(5) + Name
    size_t packet_size = 1 + 5 + name_len;
    uint8_t *packet = k_malloc(packet_size);
    if (!packet) {
        LOG_ERR("Failed to allocate packet buffer");
        return -ENOMEM;
    }

    // Build packet
    size_t idx = 0;
    packet[idx++] = RENAME_PACKET_HEADER;  // 0xBC

    // Get 64-bit timestamp
    int64_t timestamp_64 = k_uptime_get();
    
    // Extract middle 5 bytes from the 64-bit timestamp
    // Skip the most significant 3 bytes and least significant byte
    for (int i = 0; i < 5; i++) {
        packet[idx++] = (timestamp_64 >> ((4-i) * 8)) & 0xFF;
    }

    // Add name
    memcpy(&packet[idx], new_name, name_len);

    // Send packet using the discovered characteristic handle
    int err = bt_gatt_write_without_response(current_conn, 
                                           subscribe_params.value_handle,
                                           packet, 
                                           packet_size,
                                           false);
    if (err) {
        LOG_ERR("Failed to send rename packet (err %d)", err);
    } else {
        LOG_INF("Sent rename packet to '%s': new_name='%s', timestamp: %lld", 
                target_device_name, new_name, timestamp_64);

        // Get current connection address
        const bt_addr_le_t *addr = bt_conn_get_dst(current_conn);
        if (!addr) {
            LOG_ERR("Could not get connection address");
            k_free(packet);
            return -EINVAL;
        }
        save_peer_name(addr, new_name);
        
    }
      
    k_free(packet);
    return err;
} 
int request_sensor_info_command(const char *target_device_name) {
    return request_sensor_info(target_device_name);
}

int request_sensor_info(const char *target_device_name) {
    if (!target_device_name) {
        LOG_ERR("Target device name not provided");
        return -EINVAL;
    }

    if (!current_conn) {
        LOG_ERR("No active connection");
        return -ENOTCONN;
    }

    // Verify we're connected to the target device
    bool is_target_device = false;
    const bt_addr_le_t *addr = bt_conn_get_dst(current_conn);
    if (!addr) {
        LOG_ERR("Could not get connection address");
        return -EINVAL;
    }

    // Check if current connection matches target name
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        if (peer_names[i].valid && 
            bt_addr_le_cmp(&peer_names[i].addr, addr) == 0) {
            if (strcmp(peer_names[i].name, target_device_name) == 0) {
                is_target_device = true;
                break;
            }
        }
    }

    if (!is_target_device) {
        LOG_ERR("Not connected to target device '%s'", target_device_name);
        return -EINVAL;
    }

    if (!notifications_enabled || !subscribe_params.value_handle) {
        LOG_ERR("Notifications not enabled or characteristic not discovered");
        return -EINVAL;
    }

    // Calculate total packet size: Header(1) + Timestamp(5)
    size_t packet_size = 1 + 5;
    uint8_t *packet = k_malloc(packet_size);
    if (!packet) {
        LOG_ERR("Failed to allocate packet buffer");
        return -ENOMEM;
    }

    size_t idx = 0;
    packet[idx++] = REQUEST_SENSOR_INFO_HEADER;  // 0xBA

    int64_t timestamp_64 = k_uptime_get();
    for (int i = 0; i < 5; i++) {
        packet[idx++] = (timestamp_64 >> ((4-i) * 8)) & 0xFF;
    }

    // Use write_params for the request
    write_params.func = write_callback;
    write_params.handle = subscribe_params.value_handle;
    write_params.offset = 0;
    write_params.data = packet;
    write_params.length = packet_size;

    int err = bt_gatt_write_without_response(current_conn, 
                                           subscribe_params.value_handle,
                                           packet, 
                                           packet_size,
                                           false);
    if (err) {
        LOG_ERR("Failed to send sensor info request (err %d)", err);
    } else {
        LOG_INF("Sent sensor info request to '%s', timestamp: %lld", 
                target_device_name, timestamp_64);
    }

    k_free(packet);
    return err;
}

int print_sensor_instances(void) {
    if (!device_info_received) {
        LOG_ERR("No device info received yet. Run 'bt sensors' first");
        return -EINVAL;
    }

    LOG_INF("Controller: %s", received_controller.controllerName);
    LOG_INF("Number of sensors: %d", received_controller.numberOfSensors);
    
    for (int i = 0; i < received_controller.numberOfSensors; i++) {
        LOG_INF("\nSensor Instance %d:", sensor_instances[i].instanceId);
        LOG_INF("  Peripheral Instance ID: %d", sensor_instances[i].peripheralInstanceID);
        LOG_INF("  Connected: %s", sensor_instances[i].sensorConnected ? "Yes" : "No");
        
        // Access metadata through the sensor instance's meta pointer
        LOG_INF("  ID: 0x%016llX", sensor_instances[i].meta->uniqueId);
        LOG_INF("  Type: 0x%04X", sensor_instances[i].meta->sensorType);
        LOG_INF("  Reading Size: %d bytes", sensor_instances[i].meta->readingSize);
        
        if (sensor_instances[i].meta->dimensionWidth > 1 || 
            sensor_instances[i].meta->dimensionHeight > 1) {
            LOG_INF("  Dimensions: %dx%d", 
                    sensor_instances[i].meta->dimensionWidth,
                    sensor_instances[i].meta->dimensionHeight);
        }
        
        // Print data type in human-readable form
        const char *type_str;
        switch (sensor_instances[i].meta->dataType) {
            case 0x01: type_str = "INT8"; break;
            case 0x02: type_str = "UINT8"; break;
            case 0x03: type_str = "INT16"; break;
            case 0x04: type_str = "UINT16"; break;
            case 0x05: type_str = "INT32"; break;
            case 0x06: type_str = "UINT32"; break;
            case 0x07: type_str = "FLOAT"; break;
            case 0x08: type_str = "DOUBLE"; break;
            default: type_str = "UNKNOWN"; break;
        }
        LOG_INF("  Data Type: %s (0x%02X)", type_str, sensor_instances[i].meta->dataType);
        
        if (sensor_instances[i].meta->dataRate > 0) {
            LOG_INF("  Data Rate: %d Hz", sensor_instances[i].meta->dataRate);
        } else {
            LOG_INF("  Data Rate: On demand");
        }

        // Print current reading and timestamp
        LOG_INF("  Last Reading: %.2f", (double)sensor_instances[i].sensorReading);
        LOG_INF("  Last Update: %lld", sensor_instances[i].unixTime);
    }
    return 0;
}

int request_sensor_data_stream(const char *target_device_name, uint8_t enable_stream) {
    if (!target_device_name) {
        LOG_ERR("Target device name not provided");
        return -EINVAL;
    }

    if (!current_conn) {
        LOG_ERR("No active connection");
        return -ENOTCONN;
    }

    if (!device_info_received) {
        LOG_ERR("No device info available. Run 'bt sensors' first");
        return -EINVAL;
    }

    // Verify we're connected to the target device
    bool is_target_device = false;
    const bt_addr_le_t *addr = bt_conn_get_dst(current_conn);
    if (!addr) {
        LOG_ERR("Could not get connection address");
        return -EINVAL;
    }

    // Check if current connection matches target name
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        if (peer_names[i].valid && 
            bt_addr_le_cmp(&peer_names[i].addr, addr) == 0) {
            if (strcmp(peer_names[i].name, target_device_name) == 0) {
                is_target_device = true;
                break;
            }
        }
    }

    if (!is_target_device) {
        LOG_ERR("Not connected to target device '%s'", target_device_name);
        return -EINVAL;
    }

    if (!notifications_enabled || !subscribe_params.value_handle) {
        LOG_ERR("Notifications not enabled or characteristic not discovered");
        return -EINVAL;
    }

    // Calculate total packet size: Header(1) + Timestamp(5) + Stream(1) + Sensor Data Blocks (4 bytes per sensor)
    size_t packet_size = 1 + 5 + 1 + (received_controller.numberOfSensors * 4);
    uint8_t *packet = k_malloc(packet_size);
    if (!packet) {
        LOG_ERR("Failed to allocate packet buffer");
        return -ENOMEM;
    }

    size_t idx = 0;
    packet[idx++] = 0xBB;  // Data Stream Request Header

    // Add timestamp (5 bytes)
    int64_t timestamp_64 = k_uptime_get();
    for (int i = 0; i < 5; i++) {
        packet[idx++] = (timestamp_64 >> ((4-i) * 8)) & 0xFF;
    }
    
    // Add stream identifier
    packet[idx++] = enable_stream;  // Stream identifier

    // Add sensor data blocks for each sensor instance
    for (int i = 0; i < received_controller.numberOfSensors; i++) {
        packet[idx++] = sensor_instances[i].instanceId;  // Instance ID
        packet[idx++] = 0x01;  // Debug enabled
        
        // Use sensor's native data rate if available, otherwise default to 64Hz
        uint16_t data_rate = sensor_instances[i].meta->dataRate > 0 ? 
                            sensor_instances[i].meta->dataRate : 64;
        
        packet[idx++] = data_rate & 0xFF;         // Data rate low byte
        packet[idx++] = (data_rate >> 8) & 0xFF;  // Data rate high byte
        
        LOG_DBG("Added sensor block: ID=%d, Rate=%dHz", 
                sensor_instances[i].instanceId, data_rate);
    }

    int err = bt_gatt_write_without_response(current_conn, 
                                           subscribe_params.value_handle,
                                           packet, 
                                           packet_size,
                                           false);
    if (err) {
        LOG_ERR("Failed to send data stream request (err %d)", err);
    } else {
        LOG_INF("Sent data stream request to '%s' for %d sensors", 
                target_device_name, received_controller.numberOfSensors);
        
        // Log details of each sensor
        for (int i = 0; i < received_controller.numberOfSensors; i++) {
            LOG_INF("Sensor %d: ID=%d, Type=0x%04X, Rate=%dHz", 
                    i,
                    sensor_instances[i].instanceId,
                    sensor_instances[i].meta->sensorType,
                    sensor_instances[i].meta->dataRate);
        }
    }

    k_free(packet);
    return err;
}

// Add this function to load peer names from settings
int load_peer_names(void)
{
    bt_addr_le_t addrs[CONFIG_BT_MAX_PAIRED];
    char name_buffer[MAX_NAME_LENGTH];
    
    // Get all bonded devices
    bond_count = 0;
    bt_foreach_bond(BT_ID_DEFAULT, list_bonds_callback, addrs);
    
    // Load names for each device
    for (int i = 0; i < bond_count; i++) {
        int err = load_peer_name(&addrs[i], name_buffer, sizeof(name_buffer));
        if (err == 0) {
            // Store in runtime array
            for (int j = 0; j < CONFIG_BT_MAX_PAIRED; j++) {
                if (!peer_names[j].valid) {
                    bt_addr_le_copy(&peer_names[j].addr, &addrs[i]);
                    strncpy(peer_names[j].name, name_buffer, MAX_NAME_LENGTH - 1);
                    peer_names[j].name[MAX_NAME_LENGTH - 1] = '\0';
                    peer_names[j].valid = true;
                    break;
                }
            }
        }
    }
    
    return 0;
}

