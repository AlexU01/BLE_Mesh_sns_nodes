import serial
import serial.tools.list_ports
import struct
import time
from datetime import datetime

# Configuration matching prj.conf
DEVICE_VID = 0x1915
DEVICE_PID = 0x5300
BAUD_RATE = 115200

# Packet Structure
# Header: 'R', 'X' (2 bytes)
# Data: Addr(2), Type(1), TS(4), Val(4), Null(1)
# Total Payload to read after RX: 12 bytes
PACKET_SIZE = 14 
PAYLOAD_SIZE = 12

SENSOR_TYPES = {
    0: "COUNTER",
    1: "TEMP (C)",
    2: "HUMIDITY (%)",
    3: "BATTERY (mV)"
}

def find_gateway_port():
    """Auto-detect the nRF5340 based on VID/PID"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == DEVICE_VID and port.pid == DEVICE_PID:
            print(f"Found device: {port.product}")
            return port.device
    return None

def process_packet(data):
    """
    Unpacks binary data:
    < : Little Endian
    H : unsigned short (2 bytes) - Address
    B : unsigned char (1 byte)   - Type
    I : unsigned int (4 bytes)   - Timestamp
    i : signed int (4 bytes)     - Value
    x : pad byte (1 byte)        - Null terminator
    """
    try:
        addr, s_type, ts, val = struct.unpack('<HBIix', data)
        
        sensor_name = SENSOR_TYPES.get(s_type, f"TYPE_{s_type}")
        display_val = val

        if sensor_name == "TEMP (C)" or sensor_name == "HUMIDITY (%)":
            display_val = f"{val / 100.0:.2f}"

        timestamp_str = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp_str}] Node 0x{addr:04X} | {sensor_name:<12} | Val: {display_val:<8} | MeshTS: {ts}")

    except struct.error as e:
        print(f"Error unpacking: {e}")

def main():
    print("Searching for Mesh Gateway...")
    port = find_gateway_port()
    
    if not port:
        print(f"No device found with VID:0x{DEVICE_VID:04X} PID:0x{DEVICE_PID:04X}")
        return

    print(f"Connected to {port}. Listening for binary data...")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        
        while True:
            # Simple State Machine to find 'RX' header
            byte = ser.read(1)
            if byte == b'R':
                byte = ser.read(1)
                if byte == b'X':
                    # Header found, read the rest of the packet
                    payload = ser.read(PAYLOAD_SIZE)
                    if len(payload) == PAYLOAD_SIZE:
                        process_packet(payload)
                    else:
                        print("Incomplete packet")
                        
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()