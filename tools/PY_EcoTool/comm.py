import serial, threading, queue, time
from serial.tools import list_ports
from PySide6.QtCore import QObject, Signal

class Serial(QObject):
    
    on_connect = Signal(str)
    on_disconnect = Signal(str)
    on_error = Signal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.rx_queue = queue.Queue()
        self.stop_evt = threading.Event()
        self.thread = None

    # ---------- connection ----------
    def connect(self, port, baudrate, data_bits, parity, stop_bits):
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=data_bits,
                parity=parity,
                stopbits=stop_bits,
                timeout=0,
                exclusive=True,
            )
            self.on_connect.emit(port)
            self.stop_evt.clear()
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
        except Exception as e:
            self.on_error.emit(str(e))
        return True

    def disconnect(self):
        self.stop_evt.set()
        if self.thread:
            self.thread.join()

        if self.ser and self.ser.is_open:
            port = self.ser.port
            self.ser.close()
            self.on_disconnect.emit(port)

    def list_ports(self):
        return list(list_ports.comports())

    # ---------- IO ----------
    def write(self, data: bytes):
        if self.ser and self.ser.is_open:
            self.ser.write(data)

    def read(self):
        try:
            return self.rx_queue.get_nowait()
        except queue.Empty:
            return None

    # ---------- worker ----------
    def _read_loop(self):
        try:
            while not self.stop_evt.is_set():
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    if data:
                        self.rx_queue.put(data)
                else:
                    time.sleep(0.0001)  # Small delay when no data
        except Exception as e:
            self.on_error.emit(str(e))