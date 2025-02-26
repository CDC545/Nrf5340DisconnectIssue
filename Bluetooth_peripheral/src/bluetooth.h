#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <errno.h>

// Callback structure for Bluetooth events
struct bluetooth_callbacks {
    void (*connected)(void);
    void (*disconnected)(void);
    void (*data_received)(const void *data, uint16_t len);
};

// Custom service UUID
#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

// Custom characteristic UUID
#define BT_UUID_DATA_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define MAX_SENSORS 10
#define MAX_NAME_LENGTH 32

// Command to central 
#define PACKET_HEADER_DEVICE_INFO 0xAB
// Command types from central
#define CMD_REQUEST_SENSOR_INFO 0xBA
#define CMD_DATASTREAM_REQUEST 0xBB
#define CMD_RENAME_DEVICE 0xBC

extern struct bt_conn *current_conn;
extern bool is_connected;  // Add connection state tracking
extern bool notifications_enabled;  // Add notifications state tracking
extern bool send_data_enabled;  // Add data streaming state tracking

//--------------------------------------------------------------
// Bluetooth Controller
//--------------------------------------------------------------
typedef struct {
    char controllerName[16];
    char controllerAddress[16];
    uint16_t controllerType;
    float firmwareVersion;  //firware version 1.
    float batteryLevel;         // battery level in percent
    uint8_t status;              // e.g., 0 for OK, other values for error states
    uint8_t numberOfSensors;  // Track how many sensors are managed
} BluetoothController;

//--------------------------------------------------------------
// Sensor Meta Data
//--------------------------------------------------------------
typedef struct {
    uint64_t    uniqueId;              //sensor definition ID
    uint16_t   sensorType;        // Sensor type (e.g., "Temperature")
    uint8_t readingSize;     // Size in bytes of the reading
    uint8_t dimensionWidth;   // If the sensor data is multidimensional
    uint8_t dimensionHeight;   // Example fixed array for dimension sizes
    uint8_t dataType;// The data type for this sensor (int, float, etc.)
    uint16_t  dataRate;        // Data rate (e.g., Hz) or update frequency
} SensorMetaData;

//--------------------------------------------------------------
// Sensor Instance
//--------------------------------------------------------------
typedef struct {
    uint16_t  instanceId;        // Unique ID for this particular sensor occurrence
    // Dynamic reading data:
    float sensorReading;    // Simplified to a single float for demonstration
    time_t unixTime;        // Timestamp of the last reading
    
    // References to the meta data and controller:
    SensorMetaData      *meta;        // Points to the sensor's meta data
    BluetoothController *controller;  // Points to the Bluetooth controller
} SensorInstance;

// Function to set device information
void set_controller_info(BluetoothController *controller);
void update_sensor_instance(SensorInstance *instance, float reading);

// Function to send data
int send_data_via_bluetooth(uint8_t *payload, size_t payload_len);

// Function to initialize Bluetooth
int bluetooth_init(void);

// Function to save peer name
int save_peer_name(const bt_addr_le_t *addr, const char *name);

// Function to update advertising data
int update_advertising_data(const char *name);

// Structure to store peer device information
struct peer_name {
    bt_addr_le_t addr;
    char name[MAX_NAME_LENGTH];
    bool valid;
};

void get_device_address(void);  // Add this declaration

#endif /* BLUETOOTH_H */ 