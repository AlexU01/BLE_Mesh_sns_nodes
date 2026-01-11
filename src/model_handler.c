#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "model_handler.h"

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

// Define the Health Pub model and Attention callbacks 
static struct k_work_delayable attention_blink_work;
static bool attention;

static struct model_cbs mdl_cbs = {
    .gw_cb = NULL,
    .led_cb = NULL
};

static void attention_blink(struct k_work *work) {
	static int idx;
	const uint8_t pattern[] = {
		BIT(0) | BIT(1),
		BIT(1) | BIT(2),
		BIT(2) | BIT(3),
		BIT(3) | BIT(0),
	};

	if (attention) {
        // Make sure cb isn't NULL first
        if (mdl_cbs.led_cb)
            mdl_cbs.led_cb(pattern[idx++ % ARRAY_SIZE(pattern)]);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
        if (mdl_cbs.led_cb)
            mdl_cbs.led_cb(0);
	}
}

static void attention_on(const struct bt_mesh_model *mod) {
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(const struct bt_mesh_model *mod) {
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/** 
 * Publication context - required for the model to send messages
 * Buffer length is set to 1 byte since messages will be stored in
 * a different buffer when being published.
 */
BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, 1);

// Message handlers
/**
 * @brief Handler for receiving the OP_MESSAGE
 */
static int handle_message(const struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    // static struct sensor_message sm = {0};
    // static uint8_t usb_buf[20] = {0};

    // // Verify that message has correct size
    // if (buf->len < SENSOR_PAYLOAD_LEN) {
    //     LOG_WRN("Message too short");
    //     return -EMSGSIZE;
    // }

    // // Pull the data from the buffer
    // sm.type      = net_buf_simple_pull_u8(buf);
    // sm.timestamp = net_buf_simple_pull_le32(buf);
    // sm.value     = (int32_t)(net_buf_simple_pull_le32(buf));

    // // Ignore messages from current node
    // if (bt_mesh_model_elem(model)->rt->addr != ctx->addr) {
    //     // TODO: Loopback has to be treated differently, ideally the gw node should omit publishing measurements and send data straight on the USB
    //     LOG_INF("RX 0x%04x:, %d, %u, %d", 
    //             ctx->addr, sm.type, sm.timestamp, sm.value);
    // }

    // // Send data to GW via USB if a callback was registered
    // if (mdl_cbs.gw_cb) {
    //     /*
    //     * Data is serialized in this format:
    //     * |'R'|'X'| src_addr | sns_type | ts | value |'\0'|
    //     */ 
    //     uint8_t offset = 0; 
    //     // Sync chars
    //     usb_buf[offset++] = 'R';
    //     usb_buf[offset++] = 'X';
        
    //     // Source address
    //     sys_put_le16(ctx->addr, usb_buf+offset);
    //     offset += sizeof(ctx->addr);
        
    //     // Sensor type (8bit)
    //     usb_buf[offset++] = sm.type;

    //     // Timestamp
    //     sys_put_le32(sm.timestamp, usb_buf+offset);
    //     offset += sizeof(sm.timestamp);

    //     // Sensor value
    //     sys_put_le32((uint32_t)sm.value, usb_buf+offset);
    //     offset += sizeof(sm.value);
        
    //     // End char
    //     usb_buf[offset++] = '\0';

    //     mdl_cbs.gw_cb(usb_buf, offset); // 'offset' also represents the number of bytes written at this point
    // }

    // return 0;
    static uint8_t count = 0;
    static uint8_t usb_buf[MAX_SNS_MSG_LEN + 10]; // Buffer for central, should be large enough to fit incoming message + serial protocol overhead

    // Extract count from buffer
    count = net_buf_simple_pull_u8(buf);
    
    // Ignore messages from current node
    if (bt_mesh_model_elem(model)->rt->addr == ctx->addr) {
        LOG_INF("Received own message (count: %d)", count);
    } else {
        LOG_INF("RX Batch from 0x%04x (count: %d)", ctx->addr, count);
    }

    if (mdl_cbs.gw_cb) {
        size_t offset = 0;
        struct sensor_message sm;

        // Add sync bytes
        usb_buf[offset++] = 'R';
        usb_buf[offset++] = 'X';

        // Add sender address
        sys_put_le16(ctx->addr, &(usb_buf[offset]));
        offset += sizeof(ctx->addr);

        // Add sensor readings count
        usb_buf[offset++] = count; 

        // Extract readings from buffer
        uint8_t i;
        for (i = 0; i < count; i++) {
            // Check if buffer contains enough data for a full reading
            if (buf->len < SENSOR_PAYLOAD_LEN)
                break;
            
            sm.type = net_buf_simple_pull_u8(buf);
            sm.timestamp = net_buf_simple_pull_le32(buf);
            sm.value = (int32_t)net_buf_simple_pull_le32(buf);

            // Pack data into usb buffer
            usb_buf[offset++] = sm.type;
            sys_put_le32(sm.timestamp, &usb_buf[offset]); 
            offset += sizeof(sm.timestamp);
            sys_put_le32((uint32_t)sm.value, &usb_buf[offset]); 
            offset += sizeof(sm.value);
        }

        // Adjust count in usb_buf if the received buffer was incomplete
        if (i != count) {
            LOG_WRN("Received incomplete message");
            usb_buf[4] -= (count - i);
        }

        usb_buf[offset++] = '\0';
        mdl_cbs.gw_cb(usb_buf, offset);
    }

    return 0;
}

/**
 * Mapping of OpCodes to Handler Functions
 * 
 * Ops:
 * - Message: An array of sensor readings (serialized sensor_message structs). One extra byte on the min length since
 * the first byte of the payload represents the number of readings transmitted. 10 readings max limit. 
 * */ 
static const struct bt_mesh_model_op vnd_ops[] = {
    { BT_MESH_MODEL_OP_MESSAGE, BT_MESH_LEN_MIN(MIN_SNS_MSG_LEN), handle_message },
    BT_MESH_MODEL_OP_END,
};

// Model Definition

// Declare the standard SIG models needed for every mesh node
static struct bt_mesh_model sig_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
};

// Define a Vendor Model using the BT_MESH_MODEL_VND macro
// Args: Company ID, Model ID, Opcode List, Publication Data, User Data (NULL) 
static struct bt_mesh_model vnd_models[] = {
    BT_MESH_MODEL_VND(TEST_VND_COMPANY_ID, TEST_VND_MODEL_ID, vnd_ops, &vnd_pub, NULL),
};

// Element configuration (the node's capabilities)
static const struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, sig_models, vnd_models),
};

// Node composition
const struct bt_mesh_comp comp = {
    .cid = TEST_VND_COMPANY_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

// Public Functions
int model_handler_init(const struct model_cbs _mdl_cbs) {
	// Init the work queue object for the attention blink
	k_work_init_delayable(&attention_blink_work, attention_blink);

    // Store callbacks
    mdl_cbs.gw_cb = _mdl_cbs.gw_cb;
    mdl_cbs.led_cb = _mdl_cbs.led_cb;

	return 0;
}

int model_handler_send(struct sensor_message *msgs, uint8_t count) {
    if (vnd_pub.addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("Publication address not set! Provision and configure the node first.");
        return -EADDRNOTAVAIL;
    }

    if (count == 0) {
        LOG_WRN("Attempt to publish empty message.");
        return -EINVAL;
    }

    if (count > MAX_BATCH_SIZE) {
        LOG_ERR("Batch size %d exceeds limit %d", count, MAX_BATCH_SIZE);
        return -EOVERFLOW;
    }

    // Buffer payload len includes 1 byte for the message count
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_MODEL_OP_MESSAGE, MAX_SNS_MSG_LEN);
    // Init message with op code
    bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_MESSAGE);

    net_buf_simple_add_u8(&msg, count);

    for (uint8_t i = 0; i < count; i++) {
        net_buf_simple_add_u8(&msg, msgs[i].type);
        net_buf_simple_add_le32(&msg, msgs[i].timestamp);
        net_buf_simple_add_le32(&msg, (uint32_t)msgs[i].value);
    }

    struct bt_mesh_msg_ctx ctx = {
        .addr = vnd_pub.addr,
        .app_idx = vnd_pub.key,
        .send_ttl = vnd_pub.ttl,
        .send_rel = false,
    };

    return bt_mesh_model_send(vnd_models, &ctx, &msg, NULL, NULL);
}