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
#include <zephyr/devicetree.h>
#include <pm_config.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include "model_handler.h"
#include "flash_storage.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

// TODO: Remove this counter once data is available from sensors
static uint32_t app_counter = 0;
static struct gpio_callback btn_cb;
static enum usb_dc_status_code current_usb_status = USB_DC_DISCONNECTED;
static bool is_central = false;

#define SNS_READ_INTERVAL_MS 20000
#define SNS_THREAD_STACK_SIZE 1024
#define SNS_THREAD_PRIORITY 7
#define SENDER_THREAD_STACK_SIZE 2048
#define SENDER_THREAD_PRIORITY 7
#define ERASE_FLASH_THREAD_STACK_SIZE 1024
#define ERASE_FLASH_THREAD_PRIORITY 7
#define SNS_Q_SIZE MAX_BATCH_SIZE
K_MSGQ_DEFINE(sns_msg_q, sizeof(struct sensor_message), SNS_Q_SIZE, 4);

// Semaphore for the thread reading the sensors
K_SEM_DEFINE(sample_sem, 0, 1);
// Semaphore for the thread that handles the queued readings
K_SEM_DEFINE(send_samples_sem, 0, 1);
// Semaphore for the thread that calls the erase flash function
K_SEM_DEFINE(erase_flash_sem, 0, 1);

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
#define CENTRAL_LED  (leds[1])
#define LED_ON       1
#define LED_OFF      0

// #define SPI_FLASH_SECTOR_SIZE  CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE
// #define FLASH_PARTITION_ID     PM_DATA_STORAGE_ID
// #define FLASH_PARTITION_OFFSET PM_DATA_STORAGE_OFFSET
// const struct device *flash_dev = DEVICE_DT_GET(DT_CHOSEN(nordic_pm_ext_flash));

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

static void button_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    // Check if any of the buttons has triggered this interrupt
    // All buttons are on GPIO Port 0, so it's safe to check *dev against a single button
    if (dev == btns[0].port) {
        if (pins & BIT(btns[0].pin)) {
            // Pressing btn0 (Button 1 on the board) generates a new sample
            k_sem_give(&sample_sem);
        }

        if (pins & BIT(btns[1].pin)) {
            // Pressing btn1 (Button 2 on the board) handles existing readings stored in queue
            k_sem_give(&send_samples_sem);
        }

        if (pins & BIT(btns[2].pin)) {
            // Pressing btn2 (Button 3 on the board) clears the external flash
            k_sem_give(&erase_flash_sem);
        }
    }
}

// UUID so device is recognised, value will be the same correspond as the MCU HW UUID
static uint8_t dev_uuid[16];

// Boolean for available USB connection status - true if board is connected to a host via the MCU USB header
static volatile bool is_usb_conn = false;

// Get the CDC ACM UART device from the Overlay
const struct device *usb_uart_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static void uart_rx_cb(const struct device *dev, void *user_data) {
    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        uint8_t c;
        while (uart_fifo_read(dev, &c, 1) == 1) {
            // Simple command parse - receiving 'C' lets the node know it's now the central
            if (c == 'C') {
                LOG_INF("Received 'Set Central'");
                model_handler_est_central();
                gpio_pin_set_dt(&CENTRAL_LED, LED_ON);
                is_central = true;
            }
        }
    }
}

static void gateway_data_handler(const uint8_t *data, const uint8_t len, const uint16_t addr) {
    if (!is_central || !device_is_ready(usb_uart_dev)) {
        return;
    }

    uint8_t usb_buf[MAX_SNS_MSG_LEN + 10]; // Buffer for central, should be large enough to fit incoming message + serial protocol overhead
    
    // Copy the data into local buffer
    // The 4 byte offset is to leave space for the protocol header 
    memcpy(usb_buf+4, data, len);
    
    // Add sync bytes
    usb_buf[0] = 'R';
    usb_buf[1] = 'X';
    
    // Add sender address
    usb_buf[2] = (uint8_t)(addr >> 8); // Extract high byte
    usb_buf[3] = (uint8_t)(addr & 0xFF); // Extract low byte
    
    // Add end byte
    usb_buf[4+len] = '\0';

    // Send data over USB
    for (uint8_t i = 0; i < 4+len+1; i++) {
        uart_poll_out(usb_uart_dev, usb_buf[i]);
    }
}

static void provisioning_complete(uint16_t net_idx, uint16_t addr) {
    LOG_INF("Provisioning complete! NetIdx: 0x%04x, Address: 0x%04x", net_idx, addr);
    configure_model();
}

static void provisioning_reset(void) {
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

static void usb_status_cb(enum usb_dc_status_code cb_status, const uint8_t *param) {
    current_usb_status = cb_status;
    switch (cb_status) {
        case USB_DC_CONNECTED: 
            LOG_INF("USB Power");
            break;
        case USB_DC_CONFIGURED:
            LOG_INF("USB Configured");
            is_usb_conn = true;
            gpio_pin_set_dt(&USB_CONN_LED, LED_ON);
            
            // Start listening for 'Set Central' command
            uart_irq_rx_enable(usb_uart_dev);
            break;
        case USB_DC_RESUME:
            LOG_INF("USB Resumed");
            is_usb_conn = true;
            gpio_pin_set_dt(&USB_CONN_LED, LED_ON);
            uart_irq_rx_enable(usb_uart_dev);
            break;
        case USB_DC_DISCONNECTED:
        case USB_DC_SUSPEND:
            LOG_INF("USB Disconnected");
            is_usb_conn = false;
            gpio_pin_set_dt(&USB_CONN_LED, LED_OFF);
            gpio_pin_set_dt(&CENTRAL_LED, LED_OFF);

            // Stop listening to USB port
            uart_irq_rx_disable(usb_uart_dev);
            // Let the other devices know this node is no longer a central
            if (is_central) {
                gpio_pin_set_dt(&CENTRAL_LED, LED_OFF);
                is_central = false;
                model_handler_cancel_central();
            }
            break;
        default:
            ;
            break;
    }
}

static void enq_reading(struct k_msgq *q, const struct sensor_message *sm) {
    // TODO: Revisit and evaluate if 20ms of wait time is an acceptable value 
    int err = k_msgq_put(q, sm, K_MSEC(20));
    if (err == -ENOMSG || err == -EAGAIN) {
        // Queue is probably full. Remove an element and try again
        struct sensor_message temp;
        k_msgq_get(q, &temp, K_MSEC(50));
        if (k_msgq_put(q, sm, K_MSEC(200))) {
            LOG_WRN("Unable to add reading to queue");
        }
        else {
            LOG_DBG("Queue full, overwrote old sample");
        }
    } else {
        LOG_DBG("Sample enqueued. Count: %u", k_msgq_num_used_get(q));
    }
}

// Function that reads sensors (simulated for now) and adds received values to a queue of readings
static void gen_samples_and_enq(void) {
    static struct sensor_message msg;

    app_counter++;
    msg.timestamp = k_uptime_get_32();

    // Generate a counter reading
    msg.type = SENSOR_TYPE_COUNTER;
    msg.value = app_counter;
    // Try to queue message
    enq_reading(&sns_msg_q, &msg);

    // Generate a temperature or humidity reading
    if (app_counter % 2) {
        msg.type = SENSOR_TYPE_TEMP_C;
        msg.value = 2450 + (app_counter % 50); // For some variation in the values
        if (app_counter % 8) {
            msg.value = -msg.value;
        }
    } else {
        msg.type = SENSOR_TYPE_HUMID_PC;
        msg.value = 50 + (10 - (app_counter % 21));
    }
    enq_reading(&sns_msg_q, &msg);

    // If the queue is almost full, try to empty it
    if (k_msgq_num_free_get(&sns_msg_q) < 2) {
        k_sem_give(&send_samples_sem);
    }
}

// Thread that reads sensors periodically
static void sns_thread_f(void *p1, void *p2, void *p3) {
    int ret = 0;
    while (1) {
        ret = k_sem_take(&sample_sem, K_MSEC(SNS_READ_INTERVAL_MS));
        LOG_INF("Generating samples");
        gen_samples_and_enq();
    }
}

K_THREAD_DEFINE(sns_thread, SNS_THREAD_STACK_SIZE, sns_thread_f, 
                NULL, NULL, NULL, SNS_THREAD_PRIORITY, 0, 0);

// Thread to offload sending from the button interrupt handler
static void sender_thread_f(void *p1, void *p2, void *p3) {
    int ret = 0, err = 0;
    struct sensor_message batch[MAX_BATCH_SIZE];
    uint8_t buf[MAX_SNS_MSG_LEN];
    
    while (1) {
        ret = k_sem_take(&send_samples_sem, K_FOREVER);

        if (ret) {
            LOG_ERR("Failed to take send_sample_sem");
            continue;
        }

        // Peek to check if there is data to process
        while (k_msgq_num_used_get(&sns_msg_q) > 0) { 
            uint8_t count = 0;
    
            // Prepare Batch
            while (count < MAX_BATCH_SIZE && k_msgq_get(&sns_msg_q, &batch[count], K_MSEC(10)) == 0) {
                count++;
            }
    
            if (count) {
                if (is_central) {
                    // If the node is the central, send data over USB directly
                    memset(buf, 0, sizeof(buf));
                    buf[0] = count;
                    int offset = 1;

                    // Serialize the batch
                    for (int i = 0; i < count; i++) {
                        buf[offset++] = batch[i].type;

                        sys_put_le32(batch[i].timestamp, &buf[offset]);
                        offset += sizeof(batch[i].timestamp);

                        sys_put_le32(batch[i].value, &buf[offset]);
                        offset += sizeof(batch[i].value);
                    }

                    gateway_data_handler(buf, offset, get_central_address());
                } else if (central_exists()) {
                    // try to send data to central
                    err = model_handler_send(batch, count);
                    
                    if (err == -EBUSY) {
                        LOG_WRN("Mesh stack busy (err %d), reattempting transmission shortly", err);

                        k_msleep(1000);
                        err = model_handler_send(batch, count);
                        if (err) {
                            LOG_ERR("Retransmission failed (err %d). Storing data in flash", err);

                            for (int i = 0; i < count; i++) {
                                flash_storage_write(&batch[i]);
                            }
                        }
                    }
                } else {
                    // Node isn't connected to USB and no central is available, store readings in flash
                    LOG_INF("No connection available (USB or Mesh), storing data in flash");
                    for (int i = 0; i < count; i++) {
                        flash_storage_write(&batch[i]);
                    }
                }
            }
        }
    }
}

K_THREAD_DEFINE(sender_thread, SENDER_THREAD_STACK_SIZE, sender_thread_f, 
                NULL, NULL, NULL, SENDER_THREAD_PRIORITY, 0, 0);

static void erase_flash_thread_f(void *p1, void *p2, void *p3) {
    int err = 0;
    while (1) {
        k_sem_take(&erase_flash_sem, K_FOREVER);
        LOG_INF("Erasing flash");
        err = flash_storage_clear();
        if (err) {
            LOG_ERR("Flash erase failed (err %d)", err);
        }
    }
}

K_THREAD_DEFINE(erase_flash_thread, ERASE_FLASH_THREAD_STACK_SIZE, erase_flash_thread_f, 
                NULL, NULL, NULL, ERASE_FLASH_THREAD_PRIORITY, 0, 0);

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

    err = flash_storage_init();
    if (err) {
        LOG_ERR("Flash Storage Init Failed (%d)", err);
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

    // Configure UART Interrupt Callback
    uart_irq_callback_user_data_set(usb_uart_dev, uart_rx_cb, NULL);

    // Enable USB stack for DFU and recovery
    if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		int err = usb_enable(usb_status_cb);
		if (err) {
            LOG_ERR("USB stack init failed (err %d)", err);
			return 0;
		}
	}

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
    LOG_INF("Once provisioned, press button 1 to generate data");

    // Enable all LEDs as a sign that the app has started. This can be useful after a DFU, as 
    // the bootloader may take a while to copy the new image and launch it
    for (uint8_t i = 0; i < ARRAY_SIZE(leds); i++) {
        gpio_pin_set_dt(&(leds[i]), LED_ON);
    }
    // The LEDs can be turned off after 700ms. Blocking delay is ok atm since main isn't doing anything else after this point.
    k_msleep(700);
    for (uint8_t i = 0; i < ARRAY_SIZE(leds); i++) {
        gpio_pin_set_dt(&(leds[i]), LED_OFF);
    }
    // Turn the USB Conn LED back on if the board is connected to a host
    if (is_usb_conn)
        gpio_pin_set_dt(&USB_CONN_LED, LED_ON);
    if (is_central)
        gpio_pin_set_dt(&CENTRAL_LED, LED_ON);
        
    return 0;
}