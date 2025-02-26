#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <zephyr/bluetooth/conn.h>

#define MAX_DEVICES 20
#define MAX_NAME_LENGTH 32
#define MAX_MANUFACTURER_NAME_LENGTH 32
#define RENAME_PACKET_HEADER 0xBC
#define REQUEST_SENSOR_INFO_HEADER 0xBA
#define MAX_DEVICE_NAME_LENGTH 32
#define MAX_CONTROLLER_NAME_LEN 32
#define MAX_SENSOR_NAME_LEN 32
#define MAX_SENSORS 12

#define DATA_STREAM_RESPONSE 0xAA
#define PACKET_HEADER_DEVICE_INFO 0xAB

#define RENAME_PACKET_HEADER 0xBC
#define REQUEST_SENSOR_INFO_HEADER 0xBA
typedef struct {
    bt_addr_le_t addr;
    char name[MAX_NAME_LENGTH];
    char manufacturer_name[MAX_MANUFACTURER_NAME_LENGTH];
    int8_t rssi;
    int8_t tx_power;
    bool connected;
    bool connectable;
} BTDevice_t;

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
// Sensor Instance
//--------------------------------------------------------------
typedef struct {
    uint16_t  instanceId;        // Unique ID for this particular sensor occurrence
    // Dynamic reading data:
    uint16_t peripheralInstanceID;
    uint8_t sensorConnected;

    float sensorReading;    // Simplified to a single float for demonstration
    uint64_t unixTime;        // Timestamp of the last reading in milliseconds
    
    // References to the meta data and controller:
    SensorMetaData      *meta;        // Points to the sensor's meta data
    BluetoothController *controller;  // Points to the Bluetooth controller
} SensorInstance;

struct bluetooth_callbacks {
    void (*connected)(void);
    void (*disconnected)(void);
    void (*data_received)(const uint8_t *data, uint16_t len);
};

// Public functions
int bluetooth_init(struct bluetooth_callbacks *callbacks);
int start_scan(void);
void disconnect_current_connection(void);
struct bt_conn *get_current_conn(void);

// Device list access functions
BTDevice_t* get_found_devices(void);
int get_num_found_devices(void);

// Add these function declarations:
void print_found_devices(void);
int connect_to_device(int device_number);
int enable_notifications(void);
int disable_notifications(void);

// Add this declaration
int bluetooth_list_bonds(void);

// Function declaration
int send_rename_command(const char *target_device_name, const char *new_name);

// Add this declaration
int request_sensor_info_command(const char *target_device_name);

// Add this declaration
int print_sensor_instances(void);

// Add this declaration
int request_sensor_data_stream(const char *target_device_name, uint8_t enable_stream);

// Add these declarations near the top of the header file
extern SensorInstance sensor_instances[MAX_SENSORS];
extern BluetoothController received_controller;
extern bool notifications_enabled;

// For internal/programmatic use
int request_sensor_info(const char *target_device_name);

#endif /* BLUETOOTH_H */ 