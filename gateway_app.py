import sys
import serial
import serial.tools.list_ports
import struct
import sqlite3 as sql
from datetime import datetime
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QPlainTextEdit, QLabel, QPushButton, QHBoxLayout)
from PyQt6.QtCore import QThread, pyqtSignal

# --- Configuration ---
DEVICE_VID = 0x1915
DEVICE_PID = 0x5301
BAUD_RATE = 115200
DB_NAME = "sensor_data.db"

# Protocol Constants
HEADER_SIZE = 5     # 'R', 'X', Addr(2), Count(1)
READING_SIZE = 9    # Type(1), TS(4), Val(4)

SENSOR_TYPES = {
    0: "COUNTER",
    1: "TEMP (C)",
    2: "HUMIDITY (%)",
    3: "BATTERY (mV)"
}

class DatabaseManager:
    def __init__(self, db_name):
        self.db_name = db_name
        self.init_db()
    
    def init_db(self):
        # Create the table if it doesn't exist
        conn = sql.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                pc_timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                node_addr INTEGER,
                sensor_type INTEGER,
                value INTEGER,
                mesh_timestamp INTEGER
            )
        ''')

        conn.commit()
        conn.close()
    
    def save_readings(self, addr, readings):
        # Saves a batch of readings to the DB
        # Readings will be handled as a list of dicts {'type', 'ts', 'val'}
        conn = sql.connect(self.db_name)
        cursor = conn.cursor()

        data_rows = []
        for r in readings:
            # (node_addr, sensor_type, value, mesh_timestamp)
            data_rows.append((addr, r['type'], r['val'], r['ts']))
        
        cursor.executemany('''
            INSERT INTO readings (node_addr, sensor_type, value, mesh_timestamp)
            VALUES (?, ?, ?, ?)
        ''', data_rows)
        
        conn.commit()
        conn.close()
    
    def get_count(self):
        # Returns total number of rows in the table
        conn = sql.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM readings")
        count = cursor.fetchone()[0]
        conn.close()
        return count
    
    def clear_db(self):
        # Deletes all rows and returns the number of deleted rows
        conn = sql.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute("DELETE FROM readings")
        deleted_count = cursor.rowcount
        conn.commit()
        conn.close()
        return deleted_count

class SerialWorker(QThread):
    """
    Background thread to handle Serial Port reading.
    Emits signals to the GUI to avoid freezing the main interface.
    """
    # Signals
    log_signal = pyqtSignal(str)          # For appending text to the log
    data_signal = pyqtSignal(int, list)   # (addr, list of readings)
    status_signal = pyqtSignal(bool, str) # For updating connection status (Connected?, PortName)

    def __init__(self):
        super().__init__()
        self.running = True
        self.ser = None

    def find_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == DEVICE_VID and port.pid == DEVICE_PID:
                return port.device
        return None

    def run(self):
        port_name = self.find_port()
        
        if not port_name:
            self.status_signal.emit(False, "Device not found")
            self.log_signal.emit(f"Could not find device with VID:0x{DEVICE_VID:04X} PID:0x{DEVICE_PID:04X}")
            return

        try:
            self.ser = serial.Serial(port_name, BAUD_RATE, timeout=0.1)
            self.status_signal.emit(True, port_name)
            self.log_signal.emit(f"Connected to {port_name}")

            while self.running:
                # Scan for header 'R' then 'X'
                if self.ser.read(1) == b'R':
                    if self.ser.read(1) == b'X':
                        self.process_packet()
                
        except serial.SerialException as e:
            self.status_signal.emit(False, "Error")
            self.log_signal.emit(f"Serial Error: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.status_signal.emit(False, "Disconnected")

    def process_packet(self):
        try:
            # Read Addr (2) + Count (1)
            header_data = self.ser.read(3)
            if len(header_data) != 3: return

            addr = struct.unpack('<H', header_data[0:2])[0]
            count = struct.unpack('B', header_data[2:3])[0]

            # Read Payload
            payload_len = count * READING_SIZE
            payload = self.ser.read(payload_len) # Omit the end terminator here
            
            # Flush the null terminator from the end of the protocol message
            self.ser.read(1) 

            if len(payload) != payload_len:
                self.log_signal.emit("Error: Incomplete payload")
                return

            self.parse_and_emit(addr, count, payload)

        except struct.error as e:
            self.log_signal.emit(f"Parse Error: {e}")

    def parse_and_emit(self, addr, count, data):
        # Parses binary data into a list of dictionaries and emits it.
        readings = []
        offset = 0
        
        for i in range(count):
            chunk = data[offset : offset + READING_SIZE]
            s_type, node_ts, val = struct.unpack('<BIi', chunk)
            
            readings.append({
                'type': s_type,
                'ts': node_ts,
                'val': val
            })
            offset += READING_SIZE
        
        # Emit raw data for the Main Window to handle (Log + DB)
        self.data_signal.emit(addr, readings)

    def stop(self):
        self.running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mesh Gateway Monitor")
        self.resize(600, 500)

        # Init DB
        self.db = DatabaseManager(DB_NAME)

        # UI Components
        self.status_label = QLabel("Status: Idle")
        self.status_label.setStyleSheet("font-weight: bold; color: gray;")
        
        self.log_area = QPlainTextEdit()
        self.log_area.setReadOnly(True)
        # Monospace font for alignment
        font = self.log_area.font()
        font.setFamily("Courier New")
        font.setStyleHint(font.StyleHint.Monospace)
        self.log_area.setFont(font)

        # Connection button
        self.btn_retry = QPushButton("Retry Connection")
        self.btn_retry.clicked.connect(self.start_worker)

        # DB Controls
        self.btn_count = QPushButton("Check DB Count")
        self.btn_count.clicked.connect(self.check_db_count)

        self.btn_clear = QPushButton("Clear Database")
        self.btn_clear.clicked.connect(self.clear_db_data)
        self.btn_clear.setStyleSheet("color: red;")

        # Layout
        # Button row
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.btn_retry)
        btn_layout.addWidget(self.btn_count)
        btn_layout.addWidget(self.btn_clear)

        # Main layout
        layout = QVBoxLayout()
        layout.addWidget(self.status_label)
        layout.addWidget(self.log_area)
        layout.addLayout(btn_layout)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Worker Thread
        self.worker = None
        self.start_worker()

    def start_worker(self):
        if self.worker and self.worker.isRunning():
            return

        self.log_area.appendHtml("<i>Attempting connection...</i>")
        self.worker = SerialWorker()
        self.worker.log_signal.connect(self.append_log)
        self.worker.status_signal.connect(self.update_status)
        self.worker.data_signal.connect(self.sensor_data_handler)
        self.worker.start()

    def update_status(self, connected, msg):
        if connected:
            self.status_label.setText(f"Status: Connected ({msg})")
            self.status_label.setStyleSheet("font-weight: bold; color: green;")
            self.btn_retry.setEnabled(False)
        else:
            self.status_label.setText(f"Status: Disconnected ({msg})")
            self.status_label.setStyleSheet("font-weight: bold; color: red;")
            self.btn_retry.setEnabled(True)

    def append_log(self, text):
        self.log_area.appendPlainText(text)

    def sensor_data_handler(self, addr, readings):
        # Saves data to the DB and formats it for display in the TextArea
        self.db.save_readings(addr, readings)

        # Format for display
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] Node 0x{addr:04X} | Batch Size: {len(readings)}\n"

        for i, r in enumerate(readings):
            sensor_name = SENSOR_TYPES.get(r['type'], f"TYPE_{r['type']}")
            
            # Format Value for Display
            if sensor_name == "TEMP (C)" or sensor_name == "HUMIDITY (%)":
                val_str = f"{r['val'] / 100.0:.2f}"
            else:
                val_str = f"{r['val']}"

            log_entry += f"    #{i+1}: {sensor_name:<12} | Val: {val_str:<8} | TS: {r['ts']}\n"
        
        log_entry += "-" * 40
        self.log_area.appendPlainText(log_entry)

    def check_db_count(self):
        count = self.db.get_count()
        self.log_area.appendPlainText(f">>> Database contains {count} entries.")

    def clear_db_data(self):
        deleted = self.db.clear_db()
        self.log_area.appendPlainText(f">>> Database cleared. Removed {deleted} entries.")

    def closeEvent(self, event):
        """Cleanup thread on window close"""
        if self.worker:
            self.worker.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())