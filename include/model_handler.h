/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <zephyr/bluetooth/mesh.h>

/* Using a test Company ID. Real products must use an assigned 16-bit Company ID. */
#define TEST_VND_COMPANY_ID 0xFFFF
#define TEST_VND_MODEL_ID   0x0001

/* Custom OpCode: 3-byte header for Vendor Models (C0 | ID) + Company ID */
/* We will define the OpCode logic in the .c file, but access it here */
#define BT_MESH_MODEL_OP_MESSAGE BT_MESH_MODEL_OP_3(0x01, TEST_VND_COMPANY_ID)

/* The payload we want to transmit */
struct simple_message {
    uint32_t counter;
};

/* External reference to the model composition */
extern const struct bt_mesh_comp comp;

/**
 * @brief Initialize the mesh model subsystem
 */
int model_handler_init(void);

/**
 * @brief Send the integer counter to the publication address
 * * @param counter The integer value to send
 * @return int 0 on success, or negative error code
 */
int model_handler_send(uint32_t counter);

#endif /* MODEL_HANDLER_H__ */