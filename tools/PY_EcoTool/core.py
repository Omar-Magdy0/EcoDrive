import sys
import os
import signal
from PySide6.QtCore import Signal, Slot, QObject, QByteArray, QCoreApplication

build_path = os.path.join(os.path.dirname(__file__), 'build')
sys.path.insert(0, build_path)
import aebf


class aebfStream:
    def __init__(self):
        self.aebfStream = aebf.Stream(buffer_size=32768, on_frame=self.dispatch, on_error=self.on_frame_error)
        self.handlers = {}  # Dictionary: serviceID -> handler function

    def process(self, data):
        self.aebfStream.process(data)

    def write(self, deviceID, serviceID, payload):
        return aebf.encode(deviceID, serviceID, payload)

    def register_service(self, serviceID, handler):
         self.handlers[serviceID] = handler  # Just store in dict

    def dispatch(self, deviceID, serviceID, payload):
        handler = self.handlers.get(serviceID)  # Look up handler
        if handler:
            handler(deviceID, serviceID, payload)  # Call it if found

    def on_frame_error(self, error_code):
        print(f"Stream error: {error_code}")
        pass
