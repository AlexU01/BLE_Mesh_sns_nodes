#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <zephyr/bluetooth/mesh.h>

// Test company and vendor ID
#define TEST_VND_COMPANY_ID 0xFFFF
#define TEST_VND_MODEL_ID   0x0001

// Custom OpCode: 3-byte header for Vendor Models (C0 | ID) + Company ID
#define BT_MESH_MODEL_OP_MESSAGE BT_MESH_MODEL_OP_3(0x01, TEST_VND_COMPANY_ID)

// Callback type for sending data strings to the Gateway
typedef void (*gateway_data_cb_t)(const uint8_t *data_buf, const uint8_t len);

// Sensor types that the node can read and send data for
enum sensor_type {
    SENSOR_TYPE_COUNTER = 0,
    SENSOR_TYPE_TEMP_C,
    SENSOR_TYPE_HUMID_PC,
    SENSOR_TYPE_BATT_MV,
};

// Payload structure - limited to 9 bytes for now so it can use unfragmented transmission
struct sensor_message {
    uint8_t type; // Using enum sensor_type would make this field 4 bits, as it would be considered an int
    uint32_t timestamp;
    int32_t value;
};
#define SENSOR_PAYLOAD_LEN (sizeof(uint8_t) + sizeof(uint32_t) + sizeof(int32_t))

// Mesh model composition
extern const struct bt_mesh_comp comp;

/**
 * @brief Initialize the mesh model subsystem
 */
int model_handler_init(gateway_data_cb_t _gw_cb);

/**
 * @brief Send a structured sensor message
 * @param msg Pointer to the message struct
 * @return 0 on success, or negative error code
 */
int model_handler_send(struct sensor_message *msg);

#endif /* MODEL_HANDLER_H__ */