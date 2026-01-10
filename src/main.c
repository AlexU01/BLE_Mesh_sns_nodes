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
#include <zephyr/drivers/gpio.h>
#include "model_handler.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// TODO: Remove this counter once data is available from sensors
static uint32_t app_counter = 0;
static struct k_work send_work;
static struct gpio_callback btn_cb;

// Get devicetree node identifiers for LEDs and buttons
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

#define BTN0_NODE DT_ALIAS(sw0)
#define BTN1_NODE DT_ALIAS(sw1)
#define BTN2_NODE DT_ALIAS(sw2)
#define BTN3_NODE DT_ALIAS(sw3)

static const struct gpio_dt_spec leds[] = {
    GPIO_DT_SPEC_GET(LED0_NODE, gpios),
    GPIO_DT_SPEC_GET(LED1_NODE, gpios),
    GPIO_DT_SPEC_GET(LED2_NODE, gpios),
    GPIO_DT_SPEC_GET(LED3_NODE, gpios)
};

#define USB_CONN_LED (leds[0])
#define LED_ON       1
#define LED_OFF      0

// Function that will be passed as cb to model_handler
static void led_control(uint32_t led_mask) {
    for (uint8_t i = 0; i < ARRAY_SIZE(leds); i++) {
        if (led_mask & BIT(i))
            gpio_pin_set_dt(&(leds[i]), LED_ON);
        else
            gpio_pin_set_dt(&(leds[i]), LED_OFF);
    }
}

static const struct gpio_dt_spec btns[] = {
    GPIO_DT_SPEC_GET(BTN0_NODE, gpios),
    GPIO_DT_SPEC_GET(BTN1_NODE, gpios),
    GPIO_DT_SPEC_GET(BTN2_NODE, gpios),
    GPIO_DT_SPEC_GET(BTN3_NODE, gpios)
};

static void button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Check if any of the buttons has triggered this interrupt
    // All buttons are on GPIO Port 0, so it's safe to check *dev against a single button
    if (dev == btns[0].port) {
        if (pins & BIT(btns[0].pin)) {
            // Pressing btn0 (Button 1 on the board) publishes stored data
            k_work_submit(&send_work);
        }
    }
}

// UUID so device is recognised, value will be the same correspond as the MCU HW UUID
static uint8_t dev_uuid[16];

// Boolean for available USB connection status - true if board is connected to a host via the MCU USB header
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

static void usb_status_cb(enum usb_dc_status_code cb_status, const uint8_t *param)
{
    switch (cb_status) {
        case USB_DC_CONNECTED: 
            LOG_INF("USB Power");
            break;
        case USB_DC_CONFIGURED:
            LOG_INF("USB Configured");
            is_usb_conn = true;
            gpio_pin_set_dt(&USB_CONN_LED, LED_ON);
            break;
        case USB_DC_RESUME:
            LOG_INF("USB Resumed");
            is_usb_conn = true;
            gpio_pin_set_dt(&USB_CONN_LED, LED_ON);
            break;
        case USB_DC_DISCONNECTED:
            LOG_INF("USB Disconnected");
            is_usb_conn = false;
            gpio_pin_set_dt(&USB_CONN_LED, LED_OFF);
            break;
        case USB_DC_SUSPEND:
            LOG_INF("USB Disconnected");
            is_usb_conn = false;
            gpio_pin_set_dt(&USB_CONN_LED, LED_OFF);
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

    // Check if LED and Buttons port is ready - they are all on port 0
    if (!device_is_ready(leds[0].port)) {
        LOG_ERR("LEDs and Buttons GPIO port is not ready");
        return 0;
    }

    // Configure LED pins
    for (uint8_t i = 0; i < ARRAY_SIZE(leds); i++) {
        err = gpio_pin_configure_dt(&(leds[i]), GPIO_OUTPUT_ACTIVE);
        if (err) {
            LOG_ERR("LEDs configuration failed (err %d)", err);
            return err;
        }
    }

    // Configure Button pins
    for (uint8_t i = 0; i < ARRAY_SIZE(btns); i++) {
        err = gpio_pin_configure_dt(&(btns[i]), GPIO_INPUT);
        if (err) {
            LOG_ERR("Buttons configuration failed (err %d)", err);
            return err;
        }
    }

    // Configure Button interrupts
    for (uint8_t i = 0; i < ARRAY_SIZE(btns); i++) {
        err = gpio_pin_interrupt_configure_dt(&(btns[i]), GPIO_INT_EDGE_TO_ACTIVE);
        if (err) {
            LOG_ERR("Button interrupts config failed (err %d)", err);
            return err;
        }
    } 

    // Configure Button interrupt callback
    // Create mask that includes all buttons
    uint32_t btn_mask = 0;
    for (uint8_t i = 0; i < ARRAY_SIZE(btns); i++) {
        btn_mask |= BIT(btns[i].pin);
    }
    // Init cb structure
    gpio_init_callback(&btn_cb, button_handler, btn_mask);
    // Add cb to buttons
    for (uint8_t i = 0; i < ARRAY_SIZE(btns); i++) {
        err = gpio_add_callback(btns[i].port, &btn_cb);
		if (err) {
			LOG_ERR("Cannot add callback (err %d)", err);
			return err;
		}
    }

    err = model_handler_init((struct model_cbs) {.gw_cb = gateway_data_handler, .led_cb = led_control});
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
    LOG_INF("Once provisioned, press Button 1 to send message.");

    // The LEDs are controlled in reverse logic, so they are all enabled when the board powers up.
    // This can be useful after a firmware update, since the bootloader takes a while to switch images
    // so letting the LEDs turn on after the program starts allows the user to know the device is ready.
    // The LEDs can be turned off after 700ms. Blocking delay is ok atm since main isn't doing anything else after this point.
    k_msleep(700);
    for (uint8_t i = 0; i < ARRAY_SIZE(leds); i++) {
        gpio_pin_set_dt(&(leds[i]), LED_OFF);
    }
    // Turn the USB Conn LED back on if the board is connected to a host
    if (is_usb_conn)
        gpio_pin_set_dt(&USB_CONN_LED, LED_ON);

    return 0;
}