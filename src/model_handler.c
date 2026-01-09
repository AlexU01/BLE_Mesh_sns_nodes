#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include "model_handler.h"

#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

// Define the Health Pub model and Attention callbacks 
static struct k_work_delayable attention_blink_work;
static bool attention;

static void attention_blink(struct k_work *work)
{
	static int idx;
	const uint8_t pattern[] = {
		BIT(0) | BIT(1),
		BIT(1) | BIT(2),
		BIT(2) | BIT(3),
		BIT(3) | BIT(0),
	};

	if (attention) {
		dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		dk_set_leds(DK_NO_LEDS_MSK);
	}
}

static void attention_on(const struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(const struct bt_mesh_model *mod)
{
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

// Publication context - required for the model to send messages
// Buffer length: 3 bytes for Vendor OpCode + size of message
BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, 3 + SENSOR_PAYLOAD_LEN);

// Message handlers
/**
 * @brief Handler for receiving the OP_MESSAGE
 */
static int handle_message(const struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    static struct sensor_message sm;

    // Ignore messages from current node
    if (bt_mesh_model_elem(model)->rt->addr == ctx->addr) {
        return 0;
    }

    // Verify that message has correct size
    if (buf->len < SENSOR_PAYLOAD_LEN) {
        LOG_WRN("Message too short");
        return -EMSGSIZE;
    }

    // Pull the data from the buffer
    sm.type      = net_buf_simple_pull_u8(buf);
    sm.timestamp = net_buf_simple_pull_le32(buf);
    sm.value     = (int32_t)(net_buf_simple_pull_le32(buf));

    LOG_INF("RX 0x%04x:, %d, %u, %d", 
            ctx->addr, sm.type, sm.timestamp, sm.value);

    return 0;
}

// Mapping of OpCodes to Handler Functions
static const struct bt_mesh_model_op vnd_ops[] = {
    { BT_MESH_MODEL_OP_MESSAGE, sizeof(uint32_t), handle_message },
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
int model_handler_init(void)
{
	// Init the work queue object for the attention blink
	k_work_init_delayable(&attention_blink_work, attention_blink);
    
	return 0;
}

int model_handler_send(struct sensor_message *payload)
{
    // Prepare the publication message
    if (vnd_pub.addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("Publication address not set! Provision the node first.");
        return -EADDRNOTAVAIL;
    }

    LOG_INF("Sending Message: %d, %u, %d", 
        payload->type, payload->timestamp, payload->value);

    // vnd_pub.msg is a pre-allocated net_buf for the publication
    struct net_buf_simple *msg = vnd_pub.msg;

    // Clean the buffer
    bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_MESSAGE);

    // Add the data
    net_buf_simple_add_u8(msg, payload->type);
    net_buf_simple_add_le32(msg, payload->timestamp);
    net_buf_simple_add_le32(msg, (uint32_t)payload->value);

    // Publish the message using the model's publication settings
    int err = bt_mesh_model_publish(vnd_models);
    if (err) {
        LOG_ERR("Failed to publish (err %d)", err);
    }

    return err;
}