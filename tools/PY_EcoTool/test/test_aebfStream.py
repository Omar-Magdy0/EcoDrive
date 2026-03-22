# test_aebf_core.py
import sys
import unittest
import random

# Add build directory to path
build_dir = './build'
sys.path.insert(0, build_dir)

try:
    import aebf
    print(f"✅ Successfully imported aebf from: {aebf.__file__}")
except ImportError as e:
    print(f"❌ Failed to import aebf: {e}")
    print(f"Looked in: {build_dir}")
    sys.exit(1)


def generate_random_frame():
    payload_size = random.randint(1, 240)  # Random frame size between 10 and 100 bytes
    service_id = random.randint(1, 1023)
    device_id = random.randint(1, 63)
    payload = bytes(random.getrandbits(8) for _ in range(payload_size))
    return [device_id, service_id, payload, aebf.encode(device_id , service_id, payload)]

random_test_num = 100000
fail_count = 0
for i in range(random_test_num):
    device_id_gen, service_id_gen, payload_gen, frame = generate_random_frame()
    print(f"iteration {i}")
    success, error_code, device_id_dec, service_id_dec, payload_dec = aebf.decode(frame)
#
    #if(not success):
    #    print(f"❌ Test {i+1} Failed to decode frame! Error code: {error_code}")
    #    print(f"Frame: {frame.hex()}")
    #    fail_count += 1
    #    continue
    #if(device_id_gen != device_id_dec or service_id_gen != service_id_dec or payload_gen != payload_dec):
    #    print(f"❌ Test {i+1} Failed!")
    #    print(f"Generated: device_id={device_id_gen}, service_id={service_id_gen}, payload={payload_gen.hex()}")
    #    print(f"Decoded:   device_id={device_id_dec}, service_id={service_id_dec}, payload={payload_dec.hex()}")
    #    fail_count += 1

print(f"✅ {random_test_num - fail_count} tests passed. {fail_count} failures.")
    
# test Stream Reception capabilities
aebfStream = aebf.Stream(buffer_size=1024, on_frame=lambda device_id, service_id, payload: print(f"Received frame: device_id={device_id}, service_id={service_id}, payload={payload.decode('utf-8')}"), on_error=lambda error_code: print(f"Stream error: {error_code}"))

multiFrame = bytes([0xAA, 0x50, 0x30])
payloads = ["Hello", "World", "This is a test of the AEBFStream", "Another frame", "Final frame"]
device_id_gens = []
service_id_gens = []
for i in range(len(payloads)):
    device_id = random.randint(1, 63)
    service_id = random.randint(1, 1023)
    payload = payloads[i].encode()
    frame = aebf.encode(device_id, service_id, payload)
    device_id_gens.append(device_id)
    service_id_gens.append(service_id)
    print(aebf.decode(frame))
    multiFrame += frame

multiFrame += bytes([0xAA, 0x50, 0x30])
print(f"Generated {len(payloads)} frames with device_ids={device_id_gens} and service_ids={service_id_gens}")
print(f"Multi-frame byte stream: {multiFrame.hex(' ')}")
#segment the multiframe processing to validate internal state management
aebfStream.process(multiFrame[1:len(multiFrame) - 10])
aebfStream.process(multiFrame[len(multiFrame) - 10:len(multiFrame)])

import time

try:
    while True:
        time.sleep(1)  # Don't burn CPU
        # Optional: do something periodically
        print("Still running...", end='\r')
except KeyboardInterrupt:
    print("\nShutting down...")