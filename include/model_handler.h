#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <zephyr/bluetooth/mesh.h>

// Test company and vendor ID
#define TEST_VND_COMPANY_ID 0xFFFF
#define TEST_VND_MODEL_ID   0x0001

// Custom OpCode: 3-byte header for Vendor Models (C0 | ID) + Company ID
#define BT_MESH_MODEL_OP_MESSAGE BT_MESH_MODEL_OP_3(0x01, TEST_VND_COMPANY_ID)
#define BT_MESH_MODEL_OP_EST_CENTRAL BT_MESH_MODEL_OP_3(0x02, TEST_VND_COMPANY_ID)
#define BT_MESH_MODEL_OP_CANCEL_CENTRAL BT_MESH_MODEL_OP_3(0x03, TEST_VND_COMPANY_ID)

// Maximum amount of readings in a single transmission
#define MAX_BATCH_SIZE 10

// Callback type for sending data to the Gateway
typedef void (*gateway_data_cb_t)(const uint8_t *data_buf, const uint8_t len, const uint16_t addr);

// Callback for LED control
typedef void (*led_control_cb_t)(uint32_t led_mask);

// Structure to hold and pass the callbacks
struct model_cbs {
    /**
     * This callback is used by all nodes whenever a packet is received, but it only has an effect
     * if the node also acts as a gateway (is connected to a host via USB).
     */
    gateway_data_cb_t gw_cb;

    /**
     * Callback used by the health server to blink the LEDs during provisioning.
     */
    led_control_cb_t led_cb;
};

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
// Minimum length of over the air message - 1 byte for count + 1 sensor reading
#define MIN_SNS_MSG_LEN (1 + SENSOR_PAYLOAD_LEN)
#define MAX_SNS_MSG_LEN (1 + (MAX_BATCH_SIZE * SENSOR_PAYLOAD_LEN))

// Mesh model composition
extern const struct bt_mesh_comp comp;

/**
 * @brief Initialize the mesh model subsystem
 * @param _mdl_cbs Struct containing function pointers to the callback implementations
 * @return 0 on success, negative error code otherwise
 */
int model_handler_init(const struct model_cbs _mdl_cbs);

/**
 * @brief Send a structured sensor message
 * @param msgs Array of sensor messages
 * @param count Number of messages in the array
 * @return 0 on success, or negative error code
 */
int model_handler_send(struct sensor_message *msgs, uint8_t count);

/**
 * @brief Establish if a node becomes a central when it's connected to a USB port
 * @return 0 on success or negative error code
 */
int model_handler_est_central(void);

/**
 * @brief Called when a node is disconnected from its USB host to cancel its status as a central;
 * @return on success or negative error code
 */
int model_handler_cancel_central(void);

#endif /* MODEL_HANDLER_H__ */