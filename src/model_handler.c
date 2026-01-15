#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "model_handler.h"

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

// Define the Health Pub model and Attention callbacks 
static struct k_work_delayable attention_blink_work;
static bool attention;
static uint16_t central_addr = 0x0;
static bool i_am_central = false;
static bool i_can_be_central = false;

static struct model_cbs mdl_cbs = {
    .gw_cb = NULL,
    .led_cb = NULL,
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
 * Buffer can hold a payload of 2 bytes for sync and control messages.
 * Sensor readings are published using a different buffer.
 */
BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, BT_MESH_MODEL_BUF_LEN(BT_MESH_MODEL_OP_EST_CENTRAL, 2));

// Message handlers
/**
 * @brief Handler for receiving the OP_MESSAGE
 */
static int handle_message(const struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    uint8_t gw_buf[MAX_SNS_MSG_LEN];

    // Copy received data to a local buffer
    memcpy(gw_buf, buf->data, buf->len);

    // Ignore messages from current node
    // First byte of the message is always the count (number of readings received)
    if (bt_mesh_model_elem(model)->rt->addr == ctx->addr) {
        LOG_INF("Received own message (count: %d)", gw_buf[0]);
    } else {
        LOG_INF("RX Batch from 0x%04x (count: %d)", ctx->addr, gw_buf[0]);
    }

    // Send data to be processed if the node acts as gateway
    if (mdl_cbs.gw_cb)
        mdl_cbs.gw_cb(gw_buf, buf->len, ctx->addr);

    return 0;
}

static int handle_est_central(const struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf) {
    if (central_addr != 0x0) {
        LOG_INF("A central node already exists, address will not be updated");
    } else {
        central_addr = ctx->addr;
        LOG_INF("Central has been established as 0x%04x", central_addr);
    }
    return 0;
}

static int handle_cancel_central(const struct bt_mesh_model *model,
                            struct bt_mesh_msg_ctx *ctx,
                            struct net_buf_simple *buf) {
    central_addr = 0x0;

    if (i_can_be_central) {
        // Generate a random delay before publishing an attempt to become central
        uint8_t rnd;
        bt_rand(&rnd, 1);
        k_msleep(rnd);
        
        bt_mesh_model_msg_init(vnd_pub.msg, BT_MESH_MODEL_OP_EST_CENTRAL);
        net_buf_simple_add_le16(vnd_pub.msg, 0);
        int err = bt_mesh_model_publish(vnd_models);
    }
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

int model_handler_est_central(void) {
    // Remember that this node can now act as a central if needed;
    i_can_be_central = true;

    if (vnd_pub.addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("Publication address not set! Provision and configure the node first.");
        return -EADDRNOTAVAIL;
    }

    if (central_addr == 0x0)
        i_am_central = true;

    // No data to transmit, but message buffer must be initialized with op code
    bt_mesh_model_msg_init(vnd_pub.msg, BT_MESH_MODEL_OP_EST_CENTRAL);
    net_buf_simple_add_le16(vnd_pub.msg, 0);
    int err = bt_mesh_model_publish(vnd_models);
    if (err) {
        LOG_ERR("Failed to publish (err %d)", err);
    }

    return err;
}

int model_handler_cancel_central(void) {
    int err = 0;
    i_can_be_central = false;

    // If this node was disconnected from the USB while acting as central,
    // it should announce to the other nodes that it can no longer be central
    if (i_am_central) {
        i_am_central = false;
        central_addr = 0x0;
        // No data to transmit, but message buffer must be initialized with op code
        bt_mesh_model_msg_init(vnd_pub.msg, BT_MESH_MODEL_OP_CANCEL_CENTRAL);
        net_buf_simple_add_le16(vnd_pub.msg, 0);
        int err = bt_mesh_model_publish(vnd_models);
        if (err) {
            LOG_ERR("Failed to publish (err %d)", err);
        }
    }

    return err;
}