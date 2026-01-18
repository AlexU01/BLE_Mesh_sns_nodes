# nRF-Based Bluetooth Mesh Sensor Network

This project implements a decentralized Bluetooth Mesh sensor network using the nRF Connect SDK (Zephyr). It features autonomous nodes that collect sensor data, store it locally on external flash if isolated, or transmit it via Mesh to a dynamic Central Gateway. A companion Python GUI application manages data collection and visualization.

## Description

The system consists of firmware for nRF5340/nRF52 series boards and a PC-side host application.

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

### Host App

- Language: Python 3.12+
- Framework: PyQt6 (GUI)
- Dependencies: pyserial (Comms), keyring (Security), sqlite3 (Storage)

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