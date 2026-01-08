/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include "model_handler.h"

#include <dk_buttons_and_leds.h>

LOG_MODULE_REGISTER(model_handler, LOG_LEVEL_INF);

/*
 * 0. Define the Health Pub model and Attention callbacks 
 */
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
	/* Will stop rescheduling blink timer */
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

/* * 1. Define Model Context 
 */

/* Publication context - required for the model to send messages */
static struct bt_mesh_model_pub vnd_pub;

/*
 * 2. Message Handlers 
 */

/**
 * @brief Handler for receiving the OP_MESSAGE
 */
static int handle_message(const struct bt_mesh_model *model,
                          struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    /* Check if the buffer has enough data for our uint32_t */
    if (buf->len < sizeof(uint32_t)) {
        LOG_WRN("Message too short");
        return -EMSGSIZE;
    }

    /* Pull the data from the buffer */
    uint32_t received_counter = net_buf_simple_pull_le32(buf);

    LOG_INF("Received Counter: %d from addr: 0x%04x (RSSI: %d)", 
            received_counter, ctx->addr, ctx->recv_rssi);

    return 0;
}

/* Mapping of OpCodes to Handler Functions */
static const struct bt_mesh_model_op vnd_ops[] = {
    { BT_MESH_MODEL_OP_MESSAGE, sizeof(uint32_t), handle_message },
    BT_MESH_MODEL_OP_END,
};

/*
 * 3. Model Definition
 */

/* Declare the standard SIG models needed for every mesh node */
static struct bt_mesh_model sig_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
};

/* * We use BT_MESH_MODEL_VND to define a vendor model.
 * Args: Company ID, Model ID, Opcode List, Publication Data, User Data (NULL) 
 */
static struct bt_mesh_model vnd_models[] = {
    BT_MESH_MODEL_VND(TEST_VND_COMPANY_ID, TEST_VND_MODEL_ID, vnd_ops, &vnd_pub, NULL),
};

/* The element configuration - essentially the node's capabilities */
static const struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, sig_models, vnd_models),
};

/* The complete node composition */
const struct bt_mesh_comp comp = {
    .cid = TEST_VND_COMPANY_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

/*
 * 4. Public Functions
 */

int model_handler_init(void)
{
    /* Any specific init logic goes here. 
     * For basic vendor models, the standard mesh init handles most things. */

	/* Init the work queue object for the attention blink */
	k_work_init_delayable(&attention_blink_work, attention_blink);
    
	return 0;
}

int model_handler_send(uint32_t counter)
{
    /* Prepare the publication message */
    if (vnd_pub.addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_WRN("Publication address not set! Provision the node first.");
        return -EADDRNOTAVAIL;
    }

    LOG_INF("Sending Counter: %d", counter);

    /* vnd_pub.msg is a pre-allocated net_buf for the publication */
    struct net_buf_simple *msg = vnd_pub.msg;

    /* Clean the buffer */
    bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_MESSAGE);

    /* Add the data (Little Endian uint32) */
    net_buf_simple_add_le32(msg, counter);

    /* Publish the message using the model's publication settings */
    int err = bt_mesh_model_publish(vnd_models);
    if (err) {
        LOG_ERR("Failed to publish (err %d)", err);
    }

    return err;
}