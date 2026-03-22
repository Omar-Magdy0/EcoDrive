"""Main application entry point"""
import sys
import signal

from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, 
                               QVBoxLayout, QTabWidget, QLabel, 
                               QPushButton, QMenu, QComboBox,
                               QDialog, QDialogButtonBox, QListWidget, QHBoxLayout)
from PySide6.QtGui import QAction
from PySide6.QtCore import Qt
import qdarktheme
import serial

from core import aebfStream
import comm

class Connections(QWidget):
    def __init__(self, serialObject):
        super().__init__()
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        
        # Serial section
        self.layout.addWidget(QLabel("Serial"))
        
        # Port selection with refresh
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        self.refresh_btn = QPushButton("🔄 Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        port_layout.addWidget(QLabel("Port:"))
        port_layout.addWidget(self.port_combo)
        port_layout.addWidget(self.refresh_btn)
        self.layout.addLayout(port_layout)
    
        # Baud rate
        baud_layout = QHBoxLayout()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "921600"])
        self.baud_combo.setCurrentText("115200")
        baud_layout.addWidget(QLabel("Baud Rate:"))
        baud_layout.addWidget(self.baud_combo)
        self.layout.addLayout(baud_layout)
        
        # Data bits
        data_layout = QHBoxLayout()
        self.data_combo = QComboBox()
        self.data_combo.addItems(["5", "6", "7", "8"])
        self.data_combo.setCurrentText("8")
        data_layout.addWidget(QLabel("Data Bits:"))
        data_layout.addWidget(self.data_combo)
        self.layout.addLayout(data_layout)
        
        # Parity
        parity_layout = QHBoxLayout()
        self.parity_combo = QComboBox()
        self.parity_combo.addItems(["None", "Even", "Odd", "Mark", "Space"])
        parity_layout.addWidget(QLabel("Parity:"))
        parity_layout.addWidget(self.parity_combo)
        self.layout.addLayout(parity_layout)
        
        # Stop bits
        stop_layout = QHBoxLayout()
        self.stop_combo = QComboBox()
        self.stop_combo.addItems(["1", "1.5", "2"])
        stop_layout.addWidget(QLabel("Stop Bits:"))
        stop_layout.addWidget(self.stop_combo)
        self.layout.addLayout(stop_layout)
        
        # Connect/Disconnect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.layout.addWidget(self.connect_btn)
        
        # Status
        self.status_label = QLabel("Disconnected")
        self.status_label.setStyleSheet("color: red;")
        self.layout.addWidget(self.status_label)
        
        self.layout.addStretch()
        
        # Serial instance
        self.serialComm = serialObject
        self.serialComm.on_connect.connect(self.on_connect)
        self.serialComm.on_disconnect.connect(self.on_disconnect)
        self.serialComm.on_error.connect(self.on_error)
        
        # Initial port scan
        self.refresh_ports()
    
    def refresh_ports(self):
        self.port_combo.clear()
        ports = self.serialComm.list_ports()
        for info in ports:
            self.port_combo.addItem(f"{info.name} : {info.description if info.description != 'n/a' else ""}", info.device)
    
    def toggle_connection(self):
        if self.connect_btn.text() == "Connect":
            port = self.port_combo.currentData()
            if not port:
                self.status_label.setText("No port selected")
                return
            
            # Map UI selections to Qt constants
            data_map = {"5": serial.FIVEBITS, "6": serial.SIXBITS, 
                       "7": serial.SEVENBITS, "8": serial.EIGHTBITS}
            parity_map = {"None": serial.PARITY_NONE, "Even": serial.PARITY_EVEN,
                         "Odd": serial.PARITY_ODD, "Mark": serial.PARITY_MARK,
                         "Space": serial.PARITY_SPACE}
            stop_map = {"1": serial.STOPBITS_ONE, "1.5": serial.STOPBITS_ONE_POINT_FIVE,
                       "2": serial.STOPBITS_TWO}
            
            success = self.serialComm.connect(
                port,
                int(self.baud_combo.currentText()),
                data_map[self.data_combo.currentText()],
                parity_map[self.parity_combo.currentText()],
                stop_map[self.stop_combo.currentText()]
            )
            if success:
                self.connect_btn.setText("Disconnect")
        else:
            self.serialComm.disconnect()
            self.connect_btn.setText("Connect")
    
    def on_connect(self, port):
        self.status_label.setText(f"Connected to {port}")
        self.status_label.setStyleSheet("color: green;")

    def on_disconnect(self, port):
        self.status_label.setText(f"Disconnected from {port}")
        self.status_label.setStyleSheet("color: red;")

    def on_error(self, error):
        self.status_label.setText(f"Error: {error}")
        self.status_label.setStyleSheet("color: orange;")
        self.connect_btn.setText("Connect")
        self.serialComm.disconnect()
        


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.dark_mode = True
        self.setWindowTitle("Device Dashboard")
        self.setGeometry(100, 100, 800, 500)
        qdarktheme.setup_theme()
        #Menu Bars
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        edit_menu = menubar.addMenu("Edit")
        view_menu = menubar.addMenu("View")
        setting_menu = menubar.addMenu("Settings")
        setting_menu.addAction("Toggle Theme", self.toggle_theme)
        # Toolbar for quick access
        toolbar = self.addToolBar("Connection")
        toolbar.addAction("🔌 Serial")
        
        # Status bar connection indicator
        self.statusBar = self.statusBar()
        
        # Tabs
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)
    
    def add_tab(self, widget, title):
        self.tabs.addTab(widget, title)

    def remove_tab(self, title):
        self.tabs.removeTab(self.tabs.indexOf(title))
        
    def set_status(self, message):
        self.statusBar.showMessage(message)

    def toggle_theme(self):
        self.dark_mode = not self.dark_mode
        if(self.dark_mode):
            qdarktheme.setup_theme("dark")
        else:
            qdarktheme.setup_theme("light")


from aebf_text import aebf_text
from EcoDrive import EcoDrive

from PySide6.QtCore import QTimer

communication = comm.Serial()
stream = aebfStream()

def serial_poll():
    while True:
        b = communication.read()
        if b is None:
            break
        stream.process(b)

# ============== MAIN PROGRAM ==============
def main():
    """Main entry point"""
    
    # Create Qt application (required for event loop)
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    # Create serial instance
    window = MainWindow()
    window.show()
    window.showMaximized()
        
    log = aebf_text(0x0F, "LOG", stream)
    debug = aebf_text(0x0E, "DEBUG", stream)
    ecodrive = EcoDrive(stream)

    window.add_tab(Connections(communication), "Connections")
    window.add_tab(log, "Log")
    window.add_tab(debug, "Debug")
    window.add_tab(ecodrive, "EcoDrive")
    window.set_status("Ready")

     # Run Qt event loop
    timer = QTimer()
    timer.timeout.connect(serial_poll)
    timer.start(10)  # 10 ms

    try:
        return app.exec()
    except KeyboardInterrupt:
        print("\n\nStopping...")
        return 0


if __name__ == "__main__":
    sys.exit(main())