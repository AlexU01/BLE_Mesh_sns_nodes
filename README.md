# nRF-Based Bluetooth Mesh Sensor Network

This project implements a decentralized Bluetooth Mesh sensor network using the nRF Connect SDK (Zephyr). It features autonomous nodes that collect sensor data, store it locally on external flash if isolated, or transmit it via Mesh to a dynamic Central Gateway. A companion Python GUI application manages data collection and visualization.

## Description

The system consists of firmware for nRF5340/nRF52 series boards and a PC-side host application. Both applications are designed to be simple to expand when adding new sensors to the nodes, as well as new sensor types.

The mesh topology was chosen in order to allow distant nodes to communicate with a Gateway or Central node even if they are not within radio range of eachother. As long as there is at least one Relay node within range of both, the distant node and the Central can still exchange messages.

![Mesh Topologies](./assets/mesh_topologies.svg "Mesh Topologies")

At the moment, an nRF52840DK board supports reading data from a DHT11 sensor as a demonstration, while an nRF5340DK board simulates sensor readings by periodically generating compatible data.

## Node Firmware

### Sensor Logic & Role Selection
Nodes are designed to be autonomous. They wake up periodically (or when `Button 1` is pressed) to generate sensor readings. Any node connected to the PC via USB can be designated as the Central (Gateway) by the host application. Upon selection, the node broadcasts its status to the mesh network, allowing other nodes to route data towards it.

### Data Routing
Data transmission follows a priority-based routing logic. If a node is the Central and has an active USB connection, readings are piped directly to the host via USB CDC ACM. If the node is a standard mesh participant, readings are buffered in RAM and transmitted to the Central using Batch Segmentation, which packages multiple readings into a single large message to reduce airtime. In cases where the node is isolated (no Central or Mesh available), readings are stored in a flash ring buffer on the external QSPI flash.

This logic is automatically triggered when the RAM queue is almost full (8/10 readings) to avoid having to overwrite data that is not persistently stored yet. Additionally, pressing `Button 2` on the boards also triggers this sequence. 

### Reliable Storage
To ensure data integrity during isolation, the firmware implements a custom slot-based circular buffer on the external flash. Metadata, including the write head position and total record count, is persisted in a dedicated sector to survive power cycles. The storage system automatically manages sector erasures, using a wear-leveling strategy via append-only metadata updates.

### Auto-Configuration
Designed for ease of deployment, nodes automatically bind AppKeys and subscribe to the Group Address (0xC000) upon provisioning. To prevent network congestion, retransmission is disabled at the application layer, relying instead on the robust retransmission mechanisms of the Bluetooth Mesh network layer.

Additionally, nodes that have been powered off indefinitely can seamlessly rejoin the network upon turning back on, with no additional configuration needed as long as they haven't been manually excluded from the network or the security keys haven't been updated. In these instances, nodes must be reprovisioned. 

### Maintenance
The system supports Application DFU via USB (MCUmgr) using a Composite Device setup, which exposes separate CDC ports for Data and SMP (Simple Management Protocol). Additionally, Serial Recovery (MCUboot) is available via a button-hold reset sequence for recovery scenarios.

When MCUboot is configured, it will additionally only allow firmware signed with a recognised private key to run on the MCU after a firmware update, enhancing security. The toolchain handles the firmware signing, the generation of a matching public key from the provided private key, and its inclusion into the bootloader during the build process with minimal intervention. **However**, users must ensure that they do not lose the private key used to sign the firmware that was flashed via the debugger. If subsequent binaries are not signed with the same private key, DFU will no longer work over USB and the board will have to be reflashed using the debugging port.

To ensure that the system can recover after a failed update, the DFU mechanism uses dual slots. The running application is stored in the SoC's flash and the incoming update firmware is stored in a different slot located in the board's external flash module. This ensures that MCUboot can roll back to a known state if any issues are encountered during the update process and, additionally, maintains the maximum amount of available space for the firmware by using the external flash.

## Host Application

### GUI & Connectivity
The Python/PyQt6 application serves as the control center for the network. It automatically detects the Gateway node using a custom VID/PID and establishes a connection. It provides controls to designate the connected node as the Mesh Central via a specific command protocol.

### Data Management
All incoming sensor data is visualized in a live log and simultaneously persisted into a local SQLite database for historical analysis.

### Security
To prevent unauthorized access to the database or critical settings, the application implements a local login system using the OS Keyring (Windows Credential Manager). It includes a tamper detection mechanism that forces a database wipe if the stored credentials are deleted or missing while a database file exists.

## Hardware

The project has been tested on the following development kits:

- Nordic nRF5340DK: Main development platform
    - SoC: nRF5340 Dual-Core SoC
        - App Core: 128 MHz Arm Cortex-M33 with 1 MB Flash + 512 KB RAM (Runs Application & Mesh Logic)
        - Net Core: 64 MHz Arm Cortex-M33 with 256 KB Flash + 64 KB RAM (Runs Bluetooth Controller)
    - External Storage: 8 MB MX25R6435F External QSPI NOR Flash (Used for reliable data buffering)
- Nordic nRF52840DK: Compatible as a standard mesh node, as well as a Central.
    - SoC: nRF52840 Single-Core 64 MHz Arm Cortex-M4 with FPU, 1 MB Flash + 256 KB RAM
    - External storage: Same as nRF5340DK
    - Sensors: Connected to a DHT11 Temperature & Humidity sensor via GPIO (`P1.02`)

## Software Stack

### Firmware

- SDK: nRF Connect SDK (NCS) v3.1.1
- RTOS: Zephyr
- Build System: West / CMake
- Key Modules:
    - Bluetooth Mesh: Custom Vendor Model for sensor data
    - Flash: nordic,qspi-nor driver with custom raw storage implementation
    - USB: Composite Device (CDC ACM + SMP)
    - Multithreading: Dedicated threads for sensor generation and mesh transmission/storage offloading to allow the BLE stack to function correctly alongside the application
    - MCUboot: Handles DFU and firmware signing verification when enabled

After the initialization section, the application is designed to work as much as possible using event driven threads. This ensures that Zephyr can run in a tickless manner, which allows it to put the SoC into deeper sleep states when there are no active threads, as it doesn't need a systick timer constantly running to schedule the threads.

#### Sensor readings

Sensors are periodically read by a dedicated thread. The received data is packaged as an `int32_t` in a simple struct, together with the current uptime of the node and an ID of the sensor reading type. Since the sensor type is stored as an `uint8_t`, up to 256 types of sensors can be defined. 
```C
enum sensor_type {
    SENSOR_TYPE_COUNTER = 0,
    SENSOR_TYPE_TEMP_C,
    SENSOR_TYPE_HUMID_PC,
    SENSOR_TYPE_BATT_MV,
};

struct sensor_message {
    uint8_t type;
    uint32_t timestamp;
    int32_t value;
};
```

The sensor readings are placed in a Zephyr queue. The reading thread reads the number of messages stored in the queue and, if it close to full, signals a different thread to start emptying it. This `sender thread` will then empty the queue into a local buffer of `sensor_message` structs

As described before, depending on the state of the node and the network, the `sender_thread` can decide to handle the messages in three different ways:
- If the node is the central, it serializes the data and calls a gateway handler that will send the messages to the Python app on the host PC via USB
- If the node isn't connected to a host running the Gateway App, it verifies if the network has a known Central Node. If there is one, the thread makes an API call to the Custom Vendor Model which will publish data to the Mesh
- If the node is isolated or there is no Central, the thread will use the flash storage library to write the messages to the external flash partition.  

These two threads are synchronised through the use of a corresponding semaphore. Buttons 1 and 2 trigger ISR's that "give" to the `sensor_reading` and `sender` threads, respectively. Additionally, they are on a timed wait when "taking" their semaphores, to ensure the operations happen periodically as well. 

#### BLE Mesh

On a software level, each BLE Mesh Node must consist of at least one **Element**, which defines a set of functionalities through the use of at least one **Model**. A Model essentially groups a number of operations and data structures that nodes can recognise and use to interact with eachother.  

The BLE Mesh specification offers a number of predefined Models to choose from. However, these can be quite restrictive in the way they handle data packaging and interaction between nodes. As such, a Custom Vendor Model has been implemented. This Model currently supports three Ops (operations).

```C
// Test company and vendor ID
#define TEST_VND_COMPANY_ID 0xFFFF
#define TEST_VND_MODEL_ID   0x0001

// Custom OpCode: 3-byte header for Vendor Models (C0 | ID) + Company ID
#define BT_MESH_MODEL_OP_MESSAGE BT_MESH_MODEL_OP_3(0x01, TEST_VND_COMPANY_ID)
#define BT_MESH_MODEL_OP_EST_CENTRAL BT_MESH_MODEL_OP_3(0x02, TEST_VND_COMPANY_ID)
#define BT_MESH_MODEL_OP_CANCEL_CENTRAL BT_MESH_MODEL_OP_3(0x03, TEST_VND_COMPANY_ID)

static const struct bt_mesh_model_op vnd_ops[] = {
    { BT_MESH_MODEL_OP_MESSAGE, BT_MESH_LEN_MIN(MIN_SNS_MSG_LEN), handle_message },
    { BT_MESH_MODEL_OP_EST_CENTRAL, BT_MESH_LEN_EXACT(2), handle_est_central },
    { BT_MESH_MODEL_OP_CANCEL_CENTRAL, BT_MESH_LEN_EXACT(2), handle_cancel_central },
    BT_MESH_MODEL_OP_END,
};

```

1. Message Op - A Message contains up to ten serialized `sensor_message` structs as payload, packaged in a simple protocol to allow deserialization by receiving nodes. The protocol will be described shortly.

2. Establish Central Op - Used by a node that has been selected as a central by the Gateway App to let the other nodes know that they can now publish their data to the Mesh. On reception of this Op, the nodes will store the 16-bit Mesh address of the Central.

3. Cancel Central Op - When a Central node is disconnected from its host, or the Gateway App is closed, the Central lets the other nodes know that it can no longer forward their messages. The other nodes will update this information locally and will store their data in flash until a new Central is chosen.

<p align="center">
  <img src="./assets/element_composition.svg" />
</p>

The sensor nodes in this network are all identical. They contain a single Element, which in turn contains a Configuration Server Model (mandatory), a Health Server Model (used to identify the node during provisioning) and the Custom Vendor Model.

```C
// Declare the standard SIG models needed for every mesh node
static struct bt_mesh_model sig_models[] = {
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
};

/** 
 * Publication context - required for the model to send messages
 * Buffer can hold a payload of 2 bytes for sync and control messages.
 * Sensor readings are published using a different buffer.
 */
BT_MESH_MODEL_PUB_DEFINE(vnd_pub, NULL, BT_MESH_MODEL_BUF_LEN(BT_MESH_MODEL_OP_EST_CENTRAL, 2));

// Define a Vendor Model using the BT_MESH_MODEL_VND macro
// Args: Company ID, Model ID, Opcode List, Publication Data, User Data (NULL) 
static struct bt_mesh_model vnd_models[] = {
    BT_MESH_MODEL_VND(TEST_VND_COMPANY_ID, TEST_VND_MODEL_ID, vnd_ops, &vnd_pub, NULL),
};

// Element configuration (the node's capabilities)
static const struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, sig_models, vnd_models),
};
```

The protocol used to transfer messages accross the network devides the payload into 1 byte representing the number of messages being sent, followed by groups of 9 bytes representing the serialized data. 

<p align="center">
  <img src="./assets/over_air_payload.svg" />
</p>

Upon reception by a node, the data is copied directly into an internal buffer and sent to a gateway callback. If the node is not a central, no more processing is done on the data. If the node is a central, the data is further packaged inside another protocol for the USB transmission. A two byte sync sequence (`'R'|'X'`) is added at the start, followed by the 16-bit Mesh address of the Central node. After these 4 bytes, the data recieved over BLE is copied directly into the buffer and a `'\0'` is inserted to mark the end of the transmission. At this point, the data is sent out over USB to the Gateway App, which look for the sync sequence to recognise a valid transmission.

![Payload Over USB](./assets/over_usb_payload.svg)

#### Flash storage

The firmware implements a simple Flash Ring Buffer designed to reliably store sensor data on the external QSPI flash when the node is isolated from the mesh network.

The flash partition is divided into fixed-size Slots (4KB sectors, same size as a flash page). Data is written sequentially into the active slot. When a slot becomes full, the next slot in the sequence is erased to ensure it is clean and ready for writing, implementing a circular overwriting mechanism. This ensures that the storage always contains the most recent data while respecting the hardware constraint that flash memory must be erased in blocks the size of a page before being written, as the module uses NOR flash.

To survive power cycles and reboots, the system maintains a persistent context structure, containing the current write slot, byte offset, and total record count. This metadata is stored in a dedicated Metadata Sector at the end of the partition. Instead of overwriting the metadata at the same address, which would quickly wear out the flash, new metadata records are appended to the sector.

```C
struct flash_ctx {
    uint32_t magic;
    uint32_t current_slot;           // Index 0 to USED_SLOTS-1
    uint32_t write_offset;           // Byte offset within the current slot
    uint32_t total_count;            // Total records stored
    uint8_t  slot_valid[USED_SLOTS]; // 1 if slot is full
};
```

On boot, the system scans this sector to find the latest valid record, restoring the previous state of the buffer. The metadata sector is erased only when it is completely full.

#### Firmware Update

While the the inclusion and operation of the DFU mechanism is mostly managed by Zephyr and the build system, additional configuration of the USB CDC ACM had to be enabled in order to allow the Central node to use the USB peripheral alongside the SMP server.

Zephyr has been configured to expose two ports to the host, one for the application and one for the update server.

```yml
&zephyr_udc0 {
	cdc_acm_uart0: cdc_acm_uart0 {
		compatible = "zephyr,cdc-acm-uart";
	};

	cdc_acm_uart1: cdc_acm_uart1 {
        compatible = "zephyr,cdc-acm-uart";
    };
};

/ {
	chosen {
		nordic,pm-ext-flash = &mx25r64;
		zephyr,uart-mcumgr = &cdc_acm_uart1;
	};
};
```

#### Central Node Management

As mentioned, the Central node can only be designated by the Gateway App. As soon as a Node detects a connection on the USB header, it starts listening to the port. At the moment, the Gateway App only sends single characters for control. When the node receives a `'C'` byte, it knows it has been designated Central by the host and publishes this information to the Mesh.

The Central needs to know when it should no longer forward data to the host. If either the node is physically disconnected, or if it receives a `'D'` byte from the host, it publishes to the network that it is no longer the Central. Additionally, if the connection is lost (no power is detected on the USB header), the node stops listening to the port. 

### Host App

- Language: Python 3.12+
- Framework: PyQt6 (GUI)
- Dependencies: pyserial (Comms), keyring (Security), sqlite3 (Storage)

The Python-based Gateway Application uses a multi-threaded architecture based on the PyQt6 framework to ensure the user interfac remains responsive while the serial communication is being handled.

All serial port operations (scanning, connecting, and reading data streams) are offloaded to a background `QThread`, the `SerialWorker`. This prevents the main GUI thread from freezing while waiting for I/O operations or blocking on `read()` calls.

The application ensures thread safety by avoiding direct variable access between threads. Instead, it uses Qt's Signals and Slots mechanism. 

When the worker parses a complete sensor packet, it emits a custom `data_signal` carrying the parsed objects. The main thread's slot function (`sensor_data_handler`) receives this signal and safely updates the UI widgets and database.

To send commands, like designating a Central node, the main thread emits a `write_signal`. A slot within the worker thread's context receives this signal and performs the `serial.write()` operation, ensuring that the serial resource is accessed exclusively by the worker thread.

Database interactions are encapsulated in a `DatabaseManager` class using `sqlite3`. While currently synchronous within the main thread, due to the low frequency of writes, the architecture allows for easy migration to a dedicated DB thread if throughput requirements increase. Transactions are committed in batches, as the sensor packets arrive.

## Prerequisites

### Firmware

- nRF Connect SDK for VS Code (latest version recommended): Includes the toolchain required to build, flash and debug applications based on Nordic's SDK and Zephyr, as well as GUI tools for easier project configuration (software and hardware).
- nRF Mesh Mobile App: For initial provisioning of the nodes and further configuration if needed.

### Host App

- Python 3.12 or newer.
- For dependencies, a `requirements.txt` file is included in the repo:
```shell
pip install -r requirements.txt
```

## Configuration

### Enabling DFU (Device Firmware Update)

To enable USB-based DFU and the necessary partition layout for external flash storage, specific flags must be set. **Always** perform a **clean build** and a **full flash erase** when changing project configurations.

In `prj.conf`:
```
# Config DFU
CONFIG_APP_ENABLE_DFU=y
```

In `sysbuild.conf`, uncomment the lines starting with `SB_CONFIG`:
```python
# MCUboot
SB_CONFIG_BOOTLOADER_MCUBOOT=y
# MCUboot should use external flash
SB_CONFIG_PM_EXTERNAL_FLASH_MCUBOOT_SECONDARY=y

# Config for signing with custom private key
SB_CONFIG_BOOT_SIGNATURE_KEY_FILE="\${APP_DIR}/<private_key_name>.pem"
SB_CONFIG_BOOT_SIGNATURE_TYPE_ECDSA_P256=y
```

In `pm_static.yml` (Partition Layout):
When DFU is enabled, the memory layout for the external flash must be updated. Ensure the data storage partition is placed immediately after the MCUboot partition to prevent it from being overwritten. The `mcuboot_secondary` section can be commented out if DFU is not required. In that case, the storage partition should begin at address `0x0`.

```yml
# Add this if DFU is active
mcuboot_secondary:
  address: 0x0
  device: DT_CHOSEN(nordic_pm_ext_flash)
  end_address: 0xd8000
  placement:
    align:
      start: 0x4
  region: external_flash
  share_size:
  - mcuboot_primary
  size: 0xd8000

data_storage:
  # address: 0x0
  # Switch to 0xd8000 address for data_storage if using DFU
  address: 0xd8000
  region: external_flash
  size: 0x8000
  placement:
    after: [mcuboot_secondary]
```

**Warning**: As toggling DFU (enabling/disabling MCUboot) involves changing the flash offset for the data storage partition, performing the switch means that data stored before the switch can no longer be accessed. 

### Enabling TF-M (Trusted Firmware-M)

Support for TF-M is available for compatible MCUs (e.g., nRF5340). To enable it, a new build configuration must be created in VS Code using the appropriate non-secure board target (e.g., nrf5340dk_nrf5340_cpuapp_ns instead of nrf5340dk_nrf5340_cpuapp). The build system will automatically link the TF-M secure image.

## Usage

### Nodes

1. **Flash Nodes**: Flash the firmware to the nRF boards.
2. **Provision**: Use the nRF Mesh mobile app to provision all nodes into the same network.

Note: Nodes auto-configure their Publish/Subscribe addresses to 0xC000 and also bind the default AppKey index.

3. **Connect Gateway**: Connect one board to the PC via the nRF USB port (not the J-Link port).
4. **Launch App**: Run `python gateway_app.py`.
5. **Login**: Set up an admin password (first run) or log in.
6. **Connect**: Click "Retry Connection" to find the board. The app will automatically designate it as `Central`.
7. **Monitor**: Sensor readings from all other nodes will appear in the log and be saved to the database.

The buttons on the board also offer different functionalities:
- `Button 1`: Triggers a sensor reading
- `Button 2`: Triggers a data transmission attempt
- `Button 3`: Wipes the application data stored in the external flash
- `Button 4` + `Reset` (if DFU is Enabled): Holding `Button 4` while resetting the board will make the board start in USB recovery mode, which allows the user to upload a firmware binary in case the main application encounters issues and the node can no longer be updated normally.

The LEDs offer an indication of the node's status:
- `LED 1`: Turns on when the node detects a USB connection (but is not necessarily the central)
- `LED 2`: Turns on when the node is designated as the central by the Python App
- `LED 4`: If DFU is enabled, this LED lets the user know that the board is in USB recovery mode

Additionally, all 4 LEDs will briefly flash when the application starts running on the board. This is useful especially after a firmware update is performed, as the bootloader may take a while to copy and launch the new firmware. During this period, the node cannot be used. As soon as the LEDs flash, the user can be sure that the device is operating as a node.

### Gateway App

The Gateway App is a Python-based GUI tool for managing the sensor network. It serves as the primary interface for collecting data and designating the network's Central node. The app automatically detects a compatible nRF board connected via USB, by scanning for a known combination of PID and VID. It then sends a command to designate it as the Mesh Central. This node then aggregates data from the rest of the mesh network. 

The app displays incoming sensor readings in a real-time log window and simultaneously saves them to a local SQLite database (`sensor_data.db`) for persistent storage. A set of basic built-in controls are provided to check the total record count and clear the database history. App access is granted through a local password system stored in the OS Keyring. If the credentials are tampered with, the app enforces a database wipe to protect data integrity.

How to use the App:

1. Launch the application by opening a terminal window in the project directory and running `python ./gateway_app.py`.
    - On the first run, you will be prompted to create an admin password
2. Plug an nRF node into the PC (use the nRF port on the board, **NOT** the IMCU header) and click the `Connect` button.
    - The status should change to "Connected" and the log will confirm: `>> Assigned CENTRAL role to connected node`
3. Watch the log window for incoming batches of sensor data from various nodes in the mesh
4. Use the "Check DB Count" button to see storage usage. Use "Disconnect" to release the COM port and instruct the node to stop acting as the Central gateway.
5. Use the "Print All Records" button to see all recorded data.

**Note** that if a node is already connected via USB to the PC, the app will assign that node as central on startup.

### Firmware Update

If DFU is enabled, it is highly recommended to use the [AuTerm](https://github.com/thedjnK/AuTerm/releases) utility, which offers a simple to use GUI for the MCUmgr tool that performs the actual update. DFU is available **while the application is running** on the board as well. There is no need to enter Recovery Mode as long as there are no issues with the application.

**Note** that the board **must** be flashed using the debugger if it has no firmware on it, or if it has been running a configuration with DFU disabled.

Update steps:

1. Build the application
    - If DFU has just been enabled, a clean build **must** be performed
    - If the node already runs DFU capable firmware, make sure that the private key used to sign the binary has not changed
2. Connect to the nRF USB port on the board (**NOT** the IMCU header) 
    - You should now see 2 new devices in Windows' Device Manager listed as `USB Serial Device` under the `Ports (COM & LPT)` section 
    - Unfortunately, Windows seems to ignore the custom device named assigned to the devices, but the one with the higher `COM` number is generally the SMP server
3. Open AuTerm and select the corresponding Port under the `Config` tab, with the following settings:
    - Baudrate: `115200`
    - Parity: `None`
    - Stop Bits: `1`
    - Data Bits: `8`
    - Flow control: `None`
4. Open the port
    - AuTerm may label it as `MCUboot` (in the lower side of the window)
5. Switch over to the `MCUmgr` tab, go to `Images`, leave the `State` selected as `Get` and hit `Go`
    - If the correct COM port was selected, information on the images currently present in flash memory will be listed
6. Switch to the `Upload` tab and look for the desired binary - it will be located at `<project_root>\build\sensor_base\zephyr\zephyr.signed.bin`
7. Check the `Confirm` option and hit `Go` - at this point, `AuTerm` will start uploading the binary to the board
    - You can also leave the `Test` option checked - this will make it so that the bootloader will only load the new image on the next reset, and subsequent resets will revert to the older firmware
8. Once the upload is finished, reset the board
9. Wait for the bootloader to launch the new firmware - this can take up to a minute
    - You will be able to tell that the application has started when the 4 LEDs on the board briefly flash

Updates in the **Recovery Mode** are very similar. Hold down `Button 4` while resetting the board, and `LED 4` should light up to indicate that the mode has been enabled. At this point, you can simply follow the steps above to recover the node. Note, however, that the `Get` command will reveal significantly less information regarding the firmware currently running. This is not an issue, rather it is a limitation of MCUboot.