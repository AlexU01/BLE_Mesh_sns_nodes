import serial
import serial.tools.list_ports
import struct
from datetime import datetime

# Configuration matching prj.conf
DEVICE_VID = 0x1915
DEVICE_PID = 0x5301
BAUD_RATE = 115200

# Fixed Header: 'R'(1) + 'X'(1) + Addr(2) + Count(1)
HEADER_SIZE = 5
# Per reading: Type(1) + TS(4) + Val(4)
READING_SIZE = 9

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
            print(f"Found device: {port.description}")
            return port.device
    return None

def process_batch(addr, count, data):
    timestamp_str = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp_str}] Batch from Node 0x{addr:04X} ({count} readings):")

    offset = 0
    for i in range(count):
        if offset + READING_SIZE > len(data):
            print("  [Error] Incomplete data")
            break
            
        # Unpack one reading
        # B: Type, I: TS, i: Value
        chunk = data[offset : offset + READING_SIZE]
        s_type, ts, val = struct.unpack('<BIi', chunk)
        
        sensor_name = SENSOR_TYPES.get(s_type, f"TYPE_{s_type}")
        display_val = val
        if sensor_name == "TEMP (C)" or sensor_name == "HUMIDITY (%)":
            display_val = f"{val / 100.0:.2f}"

        print(f"  #{i+1}: {sensor_name:<12} | Val: {display_val:<8} | MeshTS: {ts}")
        
        offset += READING_SIZE

def main():
    print("Searching for Mesh Gateway...")
    port = find_gateway_port()
    
    if not port:
        print("Device not found.")
        return

    print(f"Connected to {port}. Listening...")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        
        while True:
            # Look for Header 'RX'
            if ser.read(1) == b'R':
                if ser.read(1) == b'X':
                    # Read Address (2 bytes)
                    addr_bytes = ser.read(2)
                    if len(addr_bytes) != 2: 
                        continue
                    addr = struct.unpack('<H', addr_bytes)[0]
                    
                    # Read Count (1 byte)
                    count_byte = ser.read(1)
                    if len(count_byte) != 1: 
                        continue
                    count = struct.unpack('B', count_byte)[0]
                    
                    # Calculate Payload Size
                    payload_len = count * READING_SIZE
                    
                    # Read Payload + Null Terminator
                    data = ser.read(payload_len + 1)
                    
                    if len(data) == payload_len + 1:
                        # Process (exclude null terminator)
                        process_batch(addr, count, data[:-1])
                    else:
                        print("Packet incomplete")
                        
    except KeyboardInterrupt:
        pass
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()