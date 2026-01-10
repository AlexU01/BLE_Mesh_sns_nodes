/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include "model_handler.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static uint32_t app_counter = 0;
static struct k_work send_work;

// UUID so device is recognised, value will be the same correspond as the MCU HW UUID
static uint8_t dev_uuid[16];

static volatile bool is_usb_conn = false;

// Get the CDC ACM UART device from the Overlay
const struct device *usb_uart_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static void gateway_data_handler(const uint8_t *data, const uint8_t len)
{
    if (!is_usb_conn || !device_is_ready(usb_uart_dev)) {
        return;
    }

    for (uint8_t i = 0; i < len; i++) {
        uart_poll_out(usb_uart_dev, data[i]);
    }
}

static void provisioning_complete(uint16_t net_idx, uint16_t addr)
{
    LOG_INF("Provisioning complete! NetIdx: 0x%04x, Address: 0x%04x", net_idx, addr);
}

static void provisioning_reset(void)
{
    // Connection settings should be wiped by the stack when a node is reset by the provisioner
    // Rebooting should ensure that advertising starts from a clean state
    // TODO: Add network core force off before reboot if there are issues with peripherals after a restart.
    LOG_WRN("Node reset. Rebooting.");
    sys_reboot(SYS_REBOOT_WARM);
}

static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
    .complete = provisioning_complete,
    .reset = provisioning_reset,
};

// Work item to offload sending from the button interrupt handler
static void send_work_handler(struct k_work *work)
{
    struct sensor_message msg;

    app_counter++;
    msg.timestamp = k_uptime_get_32();

    if (app_counter % 2 == 0) {
        msg.type = SENSOR_TYPE_TEMP_C;
        msg.value = 2450;
    } else {
        msg.type = SENSOR_TYPE_COUNTER;
        msg.value = app_counter;
    }

    model_handler_send(&msg);
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    // If Button 1 (bit 0) is pressed
    if ((has_changed & DK_BTN1_MSK) && (button_state & DK_BTN1_MSK)) {
        k_work_submit(&send_work);
    }
}

static void usb_status_cb(enum usb_dc_status_code cb_status, const uint8_t *param)
{
    switch (cb_status) {
        case USB_DC_CONNECTED: 
            LOG_INF("USB Power");
            break;
        case USB_DC_CONFIGURED:
            LOG_INF("USB Configured");
            is_usb_conn = true;
            dk_set_led_on(DK_LED4);
            break;
        case USB_DC_RESUME:
            LOG_INF("USB Resumed");
            is_usb_conn = true;
            dk_set_led_on(DK_LED4);
            break;
        case USB_DC_DISCONNECTED:
            LOG_INF("USB Disconnected");
            is_usb_conn = false;
            dk_set_led_off(DK_LED4);
            break;
        case USB_DC_SUSPEND:
            LOG_INF("USB Disconnected");
            is_usb_conn = false;
            dk_set_led_off(DK_LED4);
            break;
        default:
            ;
            break;
    }
}

int main(void)
{
    int err;

    LOG_INF("Initializing Bare-Bones BLE Mesh Vendor Sample");

    err = dk_buttons_init(button_handler);
    if (err) {
        LOG_ERR("Button init failed (err %d)", err);
        return err;
    }

    err = dk_leds_init();
    if (err) {
        LOG_ERR("LEDs init failed (err %d)", err);
        return err;
    }

    err = model_handler_init(gateway_data_handler);
    if (err) {
        LOG_ERR("Model handler init failed (err %d)", err);
        return err;
    }

    if (!device_is_ready(usb_uart_dev)) {
        LOG_ERR("CDC ACM device not ready");
        return -ENODEV;
    }

    // Enable USB stack for DFU and recovery
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		int err = usb_enable(usb_status_cb);
		if (err) {
            LOG_ERR("USB stack init failed (err %d)", err);
			return 0;
		}
	}

    // Init work queue itme for button press
    k_work_init(&send_work, send_work_handler);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }
    LOG_INF("Bluetooth initialized");

    err = hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
    if (err < 0) {
        LOG_WRN("Failed to get device ID (err %d), using random UUID", err);
        bt_rand(dev_uuid, sizeof(dev_uuid));
    }

    // Provisioning data is populated above
    // Composition data is populated in model_handler.c
    err = bt_mesh_init(&prov, &comp);
    if (err) {
        LOG_ERR("Mesh init failed (err %d)", err);
        return err;
    }

    // Reload connection data if persistent storage is configured
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

    // Start advertising
	// This will be a no-op if settings_load() loaded provisioning info
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

    LOG_INF("Mesh initialized. Waiting for provisioning...");
    LOG_INF("Once provisioned, press Button 1 to send counter.");

    return 0;
}