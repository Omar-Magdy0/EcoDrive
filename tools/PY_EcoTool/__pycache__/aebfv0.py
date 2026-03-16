import ctypes
import platform
import sys

# Global library instance
lib = None
_lib_load_error = None

# Load the correct library for each platform
def load_library():
    """Lazily load the aebfv0 library on first use"""
    global lib, _lib_load_error
    
    if lib is not None:
        return lib
    
    if _lib_load_error is not None:
        raise RuntimeError(_lib_load_error)
    
    try:
        system = platform.system()
        
        if system == "Windows":
            lib = ctypes.CDLL("./build/aebfv0.dll")
        elif system == "Darwin":  # macOS
            lib = ctypes.CDLL("./build/libaebfv0.dylib")
        else:  # Linux
            lib = ctypes.CDLL("./build/libaebfv0.so")
        
        # Setup function signatures (SAME FOR ALL PLATFORMS)
        lib.aebp_is_frame_cplt.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16]
        lib.aebp_is_frame_cplt.restype = ctypes.c_bool

        lib.aebp_encode_frame.argtypes = [
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8, ctypes.c_uint16, 
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint8
        ]
        lib.aebp_encode_frame.restype = ctypes.c_uint8

        lib.aebp_decode_frame.argtypes = [
            ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint16,
            ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint16),
            ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8)
        ]
        lib.aebp_decode_frame.restype = ctypes.c_uint8
        
        return lib
    except OSError as e:
        _lib_load_error = f"Failed to load aebfv0 library: {e}"
        raise RuntimeError(_lib_load_error)

# Wrapper functions
def is_frame_complete(frame_data, frame_len):
    """Check if frame is complete"""
    lib = load_library()
    frame_array = (ctypes.c_uint8 * len(frame_data))(*frame_data)
    return bool(lib.aebp_is_frame_cplt(frame_array, frame_len))

def encode_frame(device_id, service_id, payload):
    """Encode a frame with the given payload"""
    lib = load_library()
    output = (ctypes.c_uint8 * (len(payload) + 7))()
    payload_array = (ctypes.c_uint8 * len(payload))(*payload)
    result = lib.aebp_encode_frame(output, device_id, service_id, 
                                   payload_array, len(payload))
    if result == 0:
        return bytes(output)
    else:
        raise Exception(f"Encoding failed with code {result}")

def decode_frame(frame_data, frame_len):
    """Decode a frame and return the extracted data"""
    lib = load_library()
    device_id = ctypes.c_uint8()
    service_id = ctypes.c_uint16()
    payload_len = ctypes.c_uint8()
    payload = (ctypes.c_uint8 * 245)()
    
    # Convert bytes to uint8 array
    frame_array = (ctypes.c_uint8 * len(frame_data))(*frame_data)

    result = lib.aebp_decode_frame(
        frame_array, frame_len,
        ctypes.byref(device_id), ctypes.byref(service_id),
        payload, ctypes.byref(payload_len)
    )
    
    if result == 0:
        return {
            'device_id': device_id.value,
            'service_id': service_id.value,
            'payload': bytes(payload[:payload_len.value])
        }
    elif result == 1:
        return {'status': 'incomplete'}
    else:
        return {'status': 'corrupted'}