#!/usr/bin/env python3
import serial
import threading
import time
import sys
from datetime import datetime

# Configuration
PORT = '/dev/ttyACM0'  # Adjust to your device
BAUD = 921600  # Baud rate doesn't matter for USB CDC

# Statistics
bytes_received = 0
packets_received = 0
start_time = time.time()
last_print = start_time

def reader_thread():
    global bytes_received, packets_received, last_print
    
    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0,  # Non-blocking
        write_timeout=0
    )
    
    print(f"Connected to {PORT}")
    print("Reading as fast as possible...")
    
    start_time = time.time()
    while True:
        # Read all available data immediately
        data = ser.read(ser.in_waiting or 1)
        if data:
            bytes_received += len(data)
            packets_received += 1
            
            # Print stats every second
            now = time.time()
            if now - last_print >= 1.0:
                elapsed = now - start_time
                rate = bytes_received / elapsed / 1024  # KB/s
                pps = packets_received / elapsed  # Packets per second
                
                print(f"\rRate: {rate:.2f} KB/s, Packets: {pps:.0f}/s, "
                      f"Avg packet: {bytes_received/packets_received:.1f} bytes", 
                      end='', flush=True)
                
                # Reset counters periodically
                if elapsed > 10:
                    bytes_received = 0
                    packets_received = 0
                    start_time = now
                    last_print = now
                    
        else:
            # Small yield when no data
            time.sleep(0.0001)

if __name__ == "__main__":
    try:
        reader_thread()
    except KeyboardInterrupt:
        print("\nDone")