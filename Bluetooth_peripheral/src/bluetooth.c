#include "bluetooth.h"
#include <zephyr/debug/thread_analyzer.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(Bluetooth_Peripheral, LOG_LEVEL_DBG);

// Forward declarations
extern BluetoothController controller;
extern SensorInstance sensor_instances[MAX_SENSORS];
void on_data_received(const uint8_t *data, uint16_t len);  // Add this declaration

// Advertising data
static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 
                   0x59, 0x00,  // Company ID (Nordic Semiconductor's ID: 0x0059)
                   0x01, 0x02, 0x03)  // Custom manufacturer data
};

// Scan response data
static struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

// Global array for peer names
static struct peer_name peer_names[CONFIG_BT_MAX_PAIRED];

// Global callbacks structure
static struct bluetooth_callbacks bt_callbacks;

bool is_connected = false;  // Add connection state variable
bool is_encrypted = false;  // Add encryption state variable
bool security_pending = false;  // Add flag to track security requests
bool notifications_enabled = false;  // Explicitly initialize to false
bool send_data_enabled = false;  // Add global streaming state variable

void send_device_info(struct bt_conn *conn);  // Forward declaration

// Function declarations
static void delete_bonds(struct bt_conn *conn);
void on_connected(struct bt_conn *conn, uint8_t err);
void on_disconnected(struct bt_conn *conn, uint8_t reason);
void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param);
void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info);

// Reset notification state on init
static void reset_states(void)
{
    notifications_enabled = false;
    is_connected = false;
    is_encrypted = false;
    security_pending = false;
    send_data_enabled = false;
}

// Empty auth callbacks for "Just Works" pairing
static void auth_cancel(struct bt_conn *conn)
{
    LOG_INF("Pairing cancelled");
}

static void auth_pairing_confirm(struct bt_conn *conn)
{
    LOG_INF("Accept pairing with device");
    bt_conn_auth_pairing_confirm(conn);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) == 0) {
        bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
        LOG_INF("Pairing completed with device %s (bonded: %d)", addr, bonded);
    } else {
        LOG_INF("Pairing completed (bonded: %d)", bonded);
    }
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    if (bt_conn_get_info(conn, &info) == 0) {
        bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
        LOG_ERR("Pairing failed with device %s (reason %d)", addr, reason);
    } else {
        LOG_ERR("Pairing failed (reason %d)", reason);
    }
}

static struct bt_conn_auth_cb auth_cb_display = {
    .cancel = auth_cancel,
    .pairing_confirm = auth_pairing_confirm
};

static struct bt_conn_auth_info_cb auth_info_cb = {
    .pairing_complete = pairing_complete,
    .pairing_failed = pairing_failed
};

static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(BT_UUID_CUSTOM_SERVICE_VAL);
static struct bt_uuid_128 data_char_uuid = BT_UUID_INIT_128(BT_UUID_DATA_CHAR_VAL);
struct bt_conn *current_conn;

// Forward declare CCC callback
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    LOG_DBG("CCC callback triggered with value: 0x%04x", value);

    // Debug log all current states
    LOG_DBG("Current states - connected: %d, encrypted: %d, security_pending: %d",
            is_connected, is_encrypted, security_pending);

    // Check if we have a valid connection
    if (!current_conn) {
        LOG_ERR("No active connection for CCC change");
        return;
    }

    // Check security level
    int err = bt_conn_get_info(current_conn, &info);
    if (err) {
        LOG_ERR("Could not get connection info (err %d)", err);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(current_conn), addr, sizeof(addr));

    // Verify encryption is established
    if (info.security.level < BT_SECURITY_L2) {
        LOG_ERR("Device %s attempted to change notifications while not encrypted", addr);
        // Force disconnect if security isn't established
        bt_conn_disconnect(current_conn, BT_HCI_ERR_AUTH_FAIL);
        return;
    }

    LOG_INF("Device %s requested to %s notifications (CCC value: 0x%04x)", 
            addr,
            value == BT_GATT_CCC_NOTIFY ? "enable" : "disable",
            value);

    bool was_enabled = notifications_enabled;
    notifications_enabled = (value == BT_GATT_CCC_NOTIFY);

    // Only log if there's an actual state change
    if (was_enabled != notifications_enabled) {
        LOG_INF("Notifications are now %s for device %s", 
                notifications_enabled ? "enabled" : "disabled",
                addr);
        
        // If notifications were disabled, stop sending data immediately
        if (!notifications_enabled) {
            LOG_INF("Stopping data transmission due to notifications being disabled");
        }

    }
}

static ssize_t read_data(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf,
                        uint16_t len,
                        uint16_t offset)
{
    struct bt_conn_info info;
    int err = bt_conn_get_info(conn, &info);
    
    if (err || info.security.level < BT_SECURITY_L2) {
        return BT_GATT_ERR(BT_ATT_ERR_AUTHENTICATION);
    }
    
    return 0;  // No data to read, just enforcing security
}

static ssize_t write_data(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    LOG_DBG("Received data, len %d", len);

    // Process received data
    on_data_received(buf, len);

    return len;
}

// Define the GATT service
BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&custom_service_uuid),
    BT_GATT_CHARACTERISTIC(&data_char_uuid.uuid,
        BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
        read_data, write_data, NULL),
    BT_GATT_CCC(ccc_cfg_changed, 
                (BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT))
);


static void security_changed(struct bt_conn *conn, bt_security_t level,
                            enum bt_security_err err)
{
    LOG_DBG("Security change callback - level: %d, err: %d", level, err);

    security_pending = false;
    if (err) {
        LOG_ERR("Security failed: level %d err %d", level, err);
        switch (err) {
            case BT_SECURITY_ERR_PIN_OR_KEY_MISSING:
                LOG_INF("No bonding exists, deleting and waiting for new pairing");
                delete_bonds(conn);
                break;
            case BT_SECURITY_ERR_AUTH_FAIL:
                LOG_INF("Authentication failed, retrying security");
                bt_conn_set_security(conn, BT_SECURITY_L2);
                break;
            case BT_SECURITY_ERR_PAIR_NOT_SUPPORTED:
            case BT_SECURITY_ERR_PAIR_NOT_ALLOWED:
                LOG_ERR("Pairing not supported or allowed by peer");
                break;
            default:
                LOG_ERR("Other security error, deleting bond and disconnecting");
                delete_bonds(conn);
                bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
                break;
        }
    } else {
        LOG_INF("Security changed: level %d", level);
        if (level >= BT_SECURITY_L2) {
            LOG_INF("Connection is now encrypted");
            is_encrypted = true;
            // Log current notification state after encryption
            LOG_INF("Current notification state after encryption: %s",
                    notifications_enabled ? "enabled" : "disabled");
        }
    }

}

static struct bt_conn_cb conn_callbacks = {
    .connected = on_connected,
    .disconnected = on_disconnected,
    .le_phy_updated = on_le_phy_updated,
    .le_data_len_updated = on_le_data_len_updated,
    .security_changed = security_changed,
};


int send_data_via_bluetooth(uint8_t *payload, size_t payload_len) {
    if (!is_connected) {
        LOG_ERR("Not connected");
        return -ENOTCONN;
    }

    if (!notifications_enabled) {
        LOG_ERR("Notifications not enabled");
        return -EINVAL;
    }

    if (payload == NULL || payload_len == 0) {
        LOG_ERR("Invalid payload");
        return -EINVAL;
    }

    if (payload_len > 247) {
        LOG_ERR("Payload exceeds maximum allowed length");
        return -EMSGSIZE;
    }



    int err = bt_gatt_notify(NULL, &custom_service.attrs[1], payload, payload_len);
    if (err) {
        LOG_ERR("Failed to send data via Bluetooth (err %d)", err);
        return err;
    }

    return 0;
}

static void print_stack_usage(const char* point) {
    thread_analyzer_print(0);  // 0 for current CPU
    LOG_DBG("Stack usage at %s:", point);
    for (int i = 0; i < 3; i++) {
        k_msleep(10);  // Give time for logs to flush
    }
}

int bluetooth_init(void)
{
    int err;
    
    // Reset all states at init
    reset_states();
    
    // Get and print device address
    bt_addr_le_t addr;
    size_t count = 1;
    
    bt_id_get(&addr, &count);
    if (count > 0) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
        LOG_INF("Device address: %s", addr_str);
        
        // Update controller address
        strncpy(controller.controllerAddress, addr_str, sizeof(controller.controllerAddress) - 1);
        controller.controllerAddress[sizeof(controller.controllerAddress) - 1] = '\0';
    }
    
    LOG_DBG("Registering Bluetooth callbacks");

    bt_conn_cb_register(&conn_callbacks);

    // Register security callbacks
    err = bt_conn_auth_cb_register(&auth_cb_display);
    if (err) {
        LOG_ERR("Failed to register auth callbacks (err %d)", err);
        return err;
    }

    err = bt_conn_auth_info_cb_register(&auth_info_cb);
    if (err) {
        LOG_ERR("Failed to register auth info callbacks (err %d)", err);
        return err;
    }

    LOG_DBG("Enabling Bluetooth");
    //print_stack_usage("before_bt_enable");
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }
    
    //print_stack_usage("after_bt_enable");
    LOG_INF("Bluetooth initialized");

    // Load settings
    settings_load();
        
    if (strlen(controller.controllerName) == 0) {
        LOG_INF("Controller name is blank, setting to default: %s", CONFIG_BT_DEVICE_NAME);
        strncpy(controller.controllerName, CONFIG_BT_DEVICE_NAME, sizeof(controller.controllerName) - 1);
        controller.controllerName[sizeof(controller.controllerName) - 1] = '\0';
    }

    // Set name before advertising
    err = bt_set_name(controller.controllerName);
    if (err) {
        LOG_ERR("Failed to set BT name (err %d)", err);
        // Continue with default name
    }
        // Update advertising with loaded name
    update_advertising_data(controller.controllerName);
    LOG_INF("Updated advertising data with name");

    return 0;
}

void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection error: %d", err);
        return;
    }
    
    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);
    is_connected = true;
    
    // Request security immediately on connection
    int sec_err = bt_conn_set_security(conn, BT_SECURITY_L2);
    if (sec_err) {
        LOG_ERR("Failed to set security (err %d)", sec_err);
    } else {
        LOG_INF("Security request sent immediately on connect");
    }
    
    // Check if we have existing bond with this device
    struct bt_conn_info info;
    err = bt_conn_get_info(conn, &info);
    if (!err) {
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
        LOG_INF("Connected to device: %s", addr);
    }
    
    if (!err) {
        LOG_INF("Initial connection interval: %.2f ms", info.le.interval * 1.25);
    }
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason: %d)", reason);
    // Reset all states
    reset_states();
    
    // Log notification state before changing it
    if (notifications_enabled) {
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        LOG_INF("Disabling notifications for disconnected device %s", addr);
    }
    
    // Log connection details that was disconnected
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Device disconnected: %s", addr);
    
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    
    int err = update_advertising_data(controller.controllerName);

    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    }
}

void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    LOG_INF("PHY updated - RX: %s, TX: %s",
            param->rx_phy == BT_GAP_LE_PHY_2M ? "2M" : "1M",
            param->tx_phy == BT_GAP_LE_PHY_2M ? "2M" : "1M");
}

void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    LOG_INF("Client requested Data Length update: TX: %d bytes (time %d us), RX: %d bytes (time %d us)",
            info->tx_max_len, info->tx_max_time, info->rx_max_len, info->rx_max_time);
}

static void delete_bonds(struct bt_conn *conn)
{
    int err;
    struct bt_conn_info info;
    
    err = bt_conn_get_info(conn, &info);
    if (err) {
        LOG_ERR("Failed to get connection info (err %d)", err);
        return;
    }
    
    err = bt_unpair(BT_ID_DEFAULT, info.le.dst);
    if (err) {
        LOG_ERR("Failed to clear bonds (err %d)", err);
    } else {
        char addr[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));
        LOG_INF("Bond deleted for device: %s", addr);
    }
}

static void print_conn_info(struct bt_conn *conn) {
    // ...
    // Get current MTU
    uint16_t mtu = bt_gatt_get_mtu(conn);
    LOG_INF("  MTU: %d bytes", mtu);
    // ...
}

void send_device_info(struct bt_conn *conn) {
    if (!notifications_enabled || !is_encrypted) {
        LOG_WRN("Cannot send device info - notifications: %d, encrypted: %d",
                notifications_enabled, is_encrypted);
        return;
    }

    // Calculate required packet size for controller and sensor metadata
    size_t packet_size = 1 + sizeof(BluetoothController) +  // +1 for header
                         (controller.numberOfSensors * sizeof(SensorMetaData));

    // Allocate buffer dynamically
    uint8_t *info_packet = k_malloc(packet_size);
    if (!info_packet) {
        LOG_ERR("Failed to allocate memory for device info packet");
        return;
    }

    // Build packet with header, controller and sensor metadata
    size_t offset = 0;

    // Add packet header
    info_packet[offset] = PACKET_HEADER_DEVICE_INFO;
    offset += 1;

    // Add controller info
    memcpy(&info_packet[offset], &controller, sizeof(BluetoothController));
    offset += sizeof(BluetoothController);

    // Add metadata for each sensor
    for (int i = 0; i < controller.numberOfSensors; i++) {
        memcpy(&info_packet[offset], sensor_instances[i].meta, sizeof(SensorMetaData));
        offset += sizeof(SensorMetaData);
    }
    
    int err = send_data_via_bluetooth(info_packet, packet_size);
    if (err) {
        LOG_ERR("Failed to send device info (err %d)", err);
    } else {
        LOG_INF("Sent device information packet:");
        LOG_INF("  Header: 0x%02X", info_packet[0]);
        LOG_INF("  Controller Name: %s", controller.controllerName);
        LOG_INF("  Number of Sensors: %d", controller.numberOfSensors);
        LOG_HEXDUMP_DBG(info_packet, packet_size, "Full Device Info Packet:");
    }

    // Free allocated memory
    k_free(info_packet);
}

int save_peer_name(const bt_addr_le_t *addr, const char *name)
{
    char key[40];
    int err;

    // If no address is provided, save as device name
    if (!addr) {
        err = settings_save_one("bt/device_name", name, strlen(name));
        if (!err) {
            strncpy(controller.controllerName, name, sizeof(controller.controllerName) - 1);
            controller.controllerName[sizeof(controller.controllerName) - 1] = '\0';
        }
        return err;
    }

    // Otherwise save as peer name
    snprintf(key, sizeof(key), "bt/dev/%02x%02x%02x%02x%02x%02x",
             addr->a.val[5], addr->a.val[4], addr->a.val[3],
             addr->a.val[2], addr->a.val[1], addr->a.val[0]);

    err = settings_save_one(key, name, strlen(name));
    if (!err) {
        for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
            if (!peer_names[i].valid || 
                bt_addr_le_cmp(&peer_names[i].addr, addr) == 0) {
                bt_addr_le_copy(&peer_names[i].addr, addr);
                strncpy(peer_names[i].name, name, sizeof(peer_names[i].name) - 1);
                peer_names[i].valid = true;
                break;
            }
        }
    }
    return err;
}

static int peer_name_settings_set(const char *key, size_t len_rd,
                                settings_read_cb read_cb, void *cb_arg)
{
    // PART 1: Handle our own device name (when key is "device_name")
    if (strcmp(key, "device_name") == 0) {
        char name[CONFIG_BT_DEVICE_NAME_MAX];
        ssize_t len = read_cb(cb_arg, name, sizeof(name) - 1);
        if (len <= 0) {
            return len;
        }
        name[len] = '\0';
        
        LOG_INF("Loading our device name from flash: %s", name);
        
        // Update our controller's name
        strncpy(controller.controllerName, name, sizeof(controller.controllerName) - 1);
        controller.controllerName[sizeof(controller.controllerName) - 1] = '\0';
        LOG_INF("Updated our device name to: %s", controller.controllerName);
        return 0;
    }
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(bt_settings, "bt", NULL, peer_name_settings_set, NULL, NULL);

// Static buffers for advertising data
static uint8_t flags[] = { BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR };
static uint8_t mfg_data[] = { 0x59, 0x00, 0x01, 0x02, 0x03 };

int update_advertising_data(const char *name)
{   
    int err;
    // Only stop/start advertising if we're not connected
    if (!is_connected) {
        // Stop advertising
        err = bt_le_adv_stop();
        if (err) {
            LOG_ERR("Failed to stop advertising (err %d)", err);
            return err;
        }
    }

        // Update advertising data
        struct bt_data new_ad[] = {
            { .type = BT_DATA_FLAGS, .data = flags, .data_len = sizeof(flags) },
            { .type = BT_DATA_NAME_COMPLETE, .data = name, .data_len = strlen(name) },
            { .type = BT_DATA_MANUFACTURER_DATA, .data = mfg_data, .data_len = sizeof(mfg_data) }
        };

     if (!is_connected) {
        // Restart advertising with new data
        err = bt_le_adv_start(BT_LE_ADV_CONN, new_ad, ARRAY_SIZE(new_ad),
                             sd, ARRAY_SIZE(sd));
        if (err) {
            LOG_ERR("Failed to start advertising with new name (err %d)", err);
            return err;
        }
    }
    

    return 0;
}

// Non-shell version for internal use
void get_device_address(void) {
    bt_addr_le_t addr;
    size_t count = 1;
    
    bt_id_get(&addr, &count);
    if (count > 0) {
        char addr_str[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
        strncpy(controller.controllerAddress, addr_str, sizeof(controller.controllerAddress));
    }
}


#define TIMESTAMP_SIZE 5
#define MAX_NAME_SIZE 16

// Data stream request constants
#define STREAM_PACKET_SIZE 14  // Size of each sensor data block
#define MAX_SENSOR_BLOCKS 3    // Maximum number of sensor blocks in request

extern bool send_data_enabled;  // Declare external from main.c

void on_data_received(const uint8_t *data, uint16_t len) 
{
    uint8_t cmd = data[0];

    // Log timestamp for debugging
    LOG_DBG("Timestamp: %02x %02x %02x %02x %02x",
            data[1], data[2], data[3], data[4], data[5]);

    switch (cmd) {
        case CMD_DATASTREAM_REQUEST: {
            // Check minimum packet length (header + timestamp + stream + at least one sensor block)
            if (len < (1 + TIMESTAMP_SIZE + 1 + 4)) {  // 4 bytes per sensor block
                LOG_ERR("Data stream request packet too short");
                return;
            }

            // Extract timestamp (for debugging)
            LOG_DBG("Timestamp: %02x %02x %02x %02x %02x",
                    data[1], data[2], data[3], data[4], data[5]);

            // Get stream enable/disable flag
            uint8_t stream_enable = data[1 + TIMESTAMP_SIZE];
            LOG_INF("Data stream %s requested", stream_enable ? "enable" : "disable");

            // Process sensor blocks
            const uint8_t *sensor_blocks = &data[1 + TIMESTAMP_SIZE + 1];
            size_t remaining_len = len - (1 + TIMESTAMP_SIZE + 1);
            size_t num_blocks = remaining_len / 4;  // Each block is 4 bytes

            for (int i = 0; i < num_blocks && i < controller.numberOfSensors; i++) {
                const uint8_t *block = &sensor_blocks[i * 4];
                
                uint8_t instance_id = block[0];
                uint8_t debug_flag = block[1];
                uint16_t data_rate = (block[3] << 8) | block[2];  // Big endian

                // Find matching sensor instance
                for (int j = 0; j < controller.numberOfSensors; j++) {
                    if (sensor_instances[j].instanceId == instance_id) {
                        // Update sensor's data rate if different
                        if (sensor_instances[j].meta->dataRate != data_rate) {
                            sensor_instances[j].meta->dataRate = data_rate;
                            LOG_INF("Updated sensor %d data rate to %d Hz", 
                                   instance_id, data_rate);
                        }
                        
                        LOG_INF("Sensor %d configuration:", instance_id);
                        LOG_INF("  Debug: %s", debug_flag ? "enabled" : "disabled");
                        LOG_INF("  Data Rate: %d Hz", data_rate);
                        break;
                    }
                }
            }

            // Update global streaming state
            send_data_enabled = stream_enable;
            LOG_INF("Data streaming %s", send_data_enabled ? "enabled" : "disabled");
            break;
        }

        case CMD_REQUEST_SENSOR_INFO: {
            LOG_INF("Received request for sensor info");
            send_device_info(current_conn);
            break;
        }
        case CMD_RENAME_DEVICE: {
            // Name starts after header and timestamp
             const uint8_t *name_data = &data[1 + TIMESTAMP_SIZE];
            uint16_t name_len = len - (1 + TIMESTAMP_SIZE); 

            // Ensure we have a name in the payload
            if (name_len == 0) {
                LOG_ERR("No name in rename command");
                return;
            }

            // Create null-terminated name string
            char new_name[MAX_NAME_SIZE];
            uint16_t copy_len = MIN(name_len, MAX_NAME_SIZE - 1);
            memcpy(new_name, name_data, copy_len);
            new_name[copy_len] = '\0';

            LOG_INF("Renaming device to: %s", new_name);
            // Check name length against CONFIG_BT_DEVICE_NAME_MAX
            if (strlen(new_name) >= CONFIG_BT_DEVICE_NAME_MAX) {
                LOG_ERR("Name too long (max %d characters)", 
                        CONFIG_BT_DEVICE_NAME_MAX - 1);
                return;
            }

            // Get our own address
            bt_addr_le_t addr;
            size_t count = 1;
            bt_id_get(&addr, &count);

            if (count == 0) {
                LOG_ERR("Could not get device address");
                return;
            }

            // Update BT device name first
            int err = bt_set_name(new_name);
            if (err) {
                LOG_ERR("Failed to set BT name (err %d)", err);
                return;
            }
            
            // Update advertising with new name
            err = update_advertising_data(new_name);
            if (err) {
                LOG_ERR("Failed to update advertising data (err %d)", err);
                return;
            }

            // Save to settings
            err = settings_save_one("bt/device_name", new_name, strlen(new_name));
            if (err) {
                LOG_ERR("Failed to save name to flash (err %d)", err);
                return;
            }

            // Update controller name
            strncpy(controller.controllerName, new_name, sizeof(controller.controllerName) - 1);
            controller.controllerName[sizeof(controller.controllerName) - 1] = '\0';

            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&addr, addr_str, sizeof(addr_str));
            LOG_INF("Device %s renamed to: %s", addr_str, new_name);
            break;
        }

        default:
            LOG_WRN("Unknown command: 0x%02x", cmd);
            break;
    }
}



