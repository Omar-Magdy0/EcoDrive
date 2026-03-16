from PySide6.QtWidgets import (QWidget, QVBoxLayout, QPlainTextEdit, 
                               QLabel, QHBoxLayout, QPushButton, QLineEdit)
from PySide6.QtCore import Qt, QObject
from PySide6.QtGui import QFont, QTextCursor
from core import aebfStream

class aebf_text(QWidget):  # Inherit from QWidget, not QObject
    def __init__(self, _serviceID: int, label: str, __aebfStream: aebfStream):
        super().__init__()
        self.serviceID = _serviceID
        self.aebfStream = __aebfStream
        
        # Main layout
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # Top label
        self.title_label = QLabel(f"Service 0x{_serviceID:X} Monitor  {label}")
        self.title_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 5px;")
        layout.addWidget(self.title_label)
        
        # Text area (terminal)
        self.terminal = QPlainTextEdit()
        self.terminal.setMaximumBlockCount(1000)
        self.terminal.setReadOnly(True)
        self.terminal.setFont(QFont("Courier New", 10))
        layout.addWidget(self.terminal, stretch=3)  # Takes 3/4 of space
        
        # Bottom section for future controls
        self.bottom_widget = QWidget()
        bottom_layout = QHBoxLayout()
        self.bottom_widget.setLayout(bottom_layout)
        
        # Placeholder for future send controls
        self.placeholder_label = QLabel("Send controls will go here")
        self.placeholder_label.setStyleSheet("color: gray; padding: 10px; border: 1px dashed gray;")
        bottom_layout.addWidget(self.placeholder_label)
        
        layout.addWidget(self.bottom_widget, stretch=1)  # Takes 1/4 of space
        
        # Auto-scroll
        self.auto_scroll = True
        
        # Register with stream
        self.aebfStream.register_service(self.serviceID, self.read)
    
    def read(self, deviceID, serviceID, payload):
        text = f"[0x{deviceID:X}][0x{serviceID:X}] {payload.decode('utf-8', errors='replace')}\n"
        self.terminal.appendPlainText(text)  # Use appendPlainText, not insert
        
        if self.auto_scroll:
            self.terminal.moveCursor(QTextCursor.End)