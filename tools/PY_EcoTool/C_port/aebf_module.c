// aebf_module.c - Clean Python wrapper for your AEBF C code
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdint.h>
#include <stdbool.h>
#include "aebfStream.h"

// Include your AEBF header (or copy the definitions)
// Assuming these are your exact functions



// ==================== Python Functions ====================

// is_frame_complete(frame: bytes) -> bool
static PyObject* py_is_frame_complete(PyObject* self, PyObject* args) {
    const char* frame;
    Py_ssize_t frame_len;
    
    if (!PyArg_ParseTuple(args, "y#", &frame, &frame_len)) {
        return NULL;
    }
    
    bool result = aebf_is_frame_cplt((uint8_t*)frame, (uint16_t)frame_len);
    return PyBool_FromLong(result);
}

// encode(device_id: int, service_id: int, payload: bytes) -> bytes
static PyObject* py_encode(PyObject* self, PyObject* args) {
    uint8_t device_id;
    uint16_t service_id;
    const char* payload;
    Py_ssize_t payload_len;
    
    if (!PyArg_ParseTuple(args, "bHy#", &device_id, &service_id, &payload, &payload_len)) {
        return NULL;
    }
    
    if (payload_len > AEBF_MAX_PAYLOAD_SIZE) {
        PyErr_SetString(PyExc_ValueError, "Payload too large");
        return NULL;
    }
    
    uint8_t output[512];
    memcpy(output + 5, payload, payload_len);  // ONLY COPY
    
    uint8_t result = aebf_encode_frame(output, device_id, service_id, payload_len);
    
    if (result != 0) {
        PyErr_SetString(PyExc_RuntimeError, "Failed to encode frame");
        return NULL;
    }
    
    return PyBytes_FromStringAndSize((const char*)output, AEBF_FRAMED_SIZE(payload_len));
}

// decode(frame: bytes) -> tuple (device_id, service_id, payload)
static PyObject* py_decode(PyObject* self, PyObject* args) {
    const char* frame;
    Py_ssize_t frame_len;
    
    if (!PyArg_ParseTuple(args, "y#", &frame, &frame_len)) {
        return NULL;
    }
    
    // Make mutable copy for COBS decode
    uint8_t frame_copy[256];
    if (!frame_copy) return PyErr_NoMemory();
    memcpy(frame_copy, frame, frame_len);
    
    uint8_t device_id;
    uint16_t service_id;
    uint8_t payload_len;
    
    uint8_t result = aebf_decode_frame(frame_copy, frame_len,
                                      &device_id, &service_id,
                                      &payload_len);
    
    if (result == 0) {
        PyObject* py_payload = PyBytes_FromStringAndSize(
            (const char*)frame_copy + 5, 
            payload_len
        );
        //free(frame_copy);
        
        return PyTuple_Pack(5,
            Py_True,
            Py_None,
            PyLong_FromLong(device_id),
            PyLong_FromLong(service_id),
            py_payload);
    } else {
        //free(frame_copy);
        return PyTuple_Pack(5,
            Py_False,
            PyLong_FromLong(result),
            Py_None, Py_None, Py_None);
    }
}

// ===================== AEBF_STREAM python Wrapper =====================

typedef struct {
    PyObject_HEAD
    aebfStream_t stream;      // Embedded C struct
    uint8_t* buffer;          // Allocated buffer (C manages this)
    uint16_t buffer_size;     // Size of allocated buffer
    PyObject* py_on_frame;    // Python callback for frames
    PyObject* py_on_error;    // Python callback for errors
} AEBFStreamObject;

// Forward declarations
static void AEBFStream_dealloc(AEBFStreamObject* self);
static PyObject* AEBFStream_new(PyTypeObject* type, PyObject* args, PyObject* kwds);
static int AEBFStream_init(AEBFStreamObject* self, PyObject* args, PyObject* kwds);

// ==================== C Callback Adapters ====================

// Called from C ISR context - bridges to Python callbacks
static void on_frame_callback(const uint8_t deviceID, const uint16_t serviceID, 
                              const uint8_t* payload, const uint16_t payload_len, 
                              void* user_data) {
    AEBFStreamObject* self = (AEBFStreamObject*)user_data;
    
    if (!self->py_on_frame || self->py_on_frame == Py_None) {
        return;  // No callback registered
    }
    
    // Create Python objects for callback arguments
    PyObject* py_device = PyLong_FromLong(deviceID);
    PyObject* py_service = PyLong_FromLong(serviceID);
    PyObject* py_payload = PyBytes_FromStringAndSize((const char*)payload, payload_len);
    
    if (!py_device || !py_service || !py_payload) {
        Py_XDECREF(py_device);
        Py_XDECREF(py_service);
        Py_XDECREF(py_payload);
        return;  // Memory error - can't raise exception in callback
    }
    
    // Acquire GIL and call Python callback
    PyGILState_STATE gstate = PyGILState_Ensure();
    PyObject* result = PyObject_CallFunctionObjArgs(self->py_on_frame, 
                                                    py_device, py_service, py_payload, NULL);
    Py_XDECREF(result);
    PyGILState_Release(gstate);
    
    Py_DECREF(py_device);
    Py_DECREF(py_service);
    Py_DECREF(py_payload);
}

static void on_error_callback(char error_code, void* user_data) {
    AEBFStreamObject* self = (AEBFStreamObject*)user_data;
    
    if (!self->py_on_error || self->py_on_error == Py_None) {
        return;
    }
    
    PyGILState_STATE gstate = PyGILState_Ensure();
    PyObject* result = PyObject_CallFunction(self->py_on_error, "i", (int)error_code);
    Py_XDECREF(result);
    PyGILState_Release(gstate);
}

// ==================== Python Methods ====================

// __init__(self, buffer_size=256, on_frame=None, on_error=None)
static int AEBFStream_init(AEBFStreamObject* self, PyObject* args, PyObject* kwds) {
    static char* kwlist[] = {"buffer_size", "on_frame", "on_error", NULL};
    
    uint16_t buffer_size = 256;  // Default buffer size
    PyObject* on_frame = Py_None;
    PyObject* on_error = Py_None;
    
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|HOO", kwlist,
                                     &buffer_size, &on_frame, &on_error)) {
        return -1;
    }
    
    // Validate buffer size
    if (buffer_size < 32) {
        PyErr_SetString(PyExc_ValueError, "Buffer size too small (minimum 32 bytes)");
        return -1;
    }
    if (buffer_size > 65535) {
        PyErr_SetString(PyExc_ValueError, "Buffer size too large (maximum 65535 bytes)");
        return -1;
    }
    
    // Validate callbacks
    if (on_frame != Py_None && !PyCallable_Check(on_frame)) {
        PyErr_SetString(PyExc_TypeError, "on_frame must be callable or None");
        return -1;
    }
    if (on_error != Py_None && !PyCallable_Check(on_error)) {
        PyErr_SetString(PyExc_TypeError, "on_error must be callable or None");
        return -1;
    }
    
    // ===== C MEMORY MANAGEMENT =====
    // Allocate buffer for the stream parser
    self->buffer = (uint8_t*)malloc(buffer_size);
    if (!self->buffer) {
        PyErr_SetString(PyExc_MemoryError, "Failed to allocate stream buffer");
        return -1;
    }
    self->buffer_size = buffer_size;
    
    // Store Python callbacks
    Py_XINCREF(on_frame);
    Py_XINCREF(on_error);
    Py_XDECREF(self->py_on_frame);
    Py_XDECREF(self->py_on_error);
    self->py_on_frame = on_frame;
    self->py_on_error = on_error;
    
    // Initialize C stream with our buffer and callbacks
    aebfStream_init(&self->stream,
                    self->buffer, self->buffer_size,
                    on_frame == Py_None ? NULL : on_frame_callback,
                    on_error == Py_None ? NULL : on_error_callback,
                    self);  // Pass self as user_data
    
    return 0;
}

// process(self, data: bytes) -> int
// Returns number of bytes processed (always len(data) unless error)
static PyObject* AEBFStream_process(AEBFStreamObject* self, PyObject* args) {
    const char* data;
    Py_ssize_t data_len;
    
    if (!PyArg_ParseTuple(args, "y#", &data, &data_len)) {
        return NULL;
    }
    
    // Process the byte stream
    aebfStream_process(&self->stream, (const uint8_t*)data, (uint16_t)data_len);
    
    Py_RETURN_NONE;
}

// reset(self) -> None
static PyObject* AEBFStream_reset(AEBFStreamObject* self, PyObject* Py_UNUSED(ignored)) {
    aebfStream_reset(&self->stream);
    Py_RETURN_NONE;
}

// clear(self) -> None
static PyObject* AEBFStream_clear(AEBFStreamObject* self, PyObject* Py_UNUSED(ignored)) {
    aebfStream_clear(&self->stream);
    Py_RETURN_NONE;
}

// get_stats(self) -> dict
static PyObject* AEBFStream_get_stats(AEBFStreamObject* self, PyObject* Py_UNUSED(ignored)) {
    uint32_t frames_rx, frames_valid, frames_corrupted, resyncs;
    
    aebfStream_get_stats(&self->stream,
                        &frames_rx, &frames_valid, &frames_corrupted, &resyncs);
    
    return Py_BuildValue("{s:I,s:I,s:I,s:I}",
                        "received", frames_rx,
                        "valid", frames_valid,
                        "corrupted", frames_corrupted,
                        "resyncs", resyncs);
}

// get_buffer_info(self) -> tuple (size, used)
static PyObject* AEBFStream_get_buffer_info(AEBFStreamObject* self, PyObject* Py_UNUSED(ignored)) {
    return Py_BuildValue("(HH)", 
                        self->buffer_size,
                        self->stream.frame_buf_end);
}

// resize_buffer(self, new_size: int) -> bool
static PyObject* AEBFStream_resize_buffer(AEBFStreamObject* self, PyObject* args) {
    uint16_t new_size;
    
    if (!PyArg_ParseTuple(args, "H", &new_size)) {
        return NULL;
    }
    
    if (new_size < 32) {
        PyErr_SetString(PyExc_ValueError, "Buffer size too small");
        return NULL;
    }
    
    // Can't shrink below current used data
    if (new_size < self->stream.frame_buf_end) {
        PyErr_SetString(PyExc_ValueError, "Cannot shrink below current used buffer size");
        return NULL;
    }
    
    // Reallocate buffer (C memory management)
    uint8_t* new_buffer = (uint8_t*)realloc(self->buffer, new_size);
    if (!new_buffer) {
        PyErr_SetString(PyExc_MemoryError, "Failed to resize buffer");
        return NULL;
    }
    
    // Update pointers
    self->buffer = new_buffer;
    self->buffer_size = new_size;
    self->stream.frame_buf = new_buffer;
    self->stream.frame_buf_size = new_size;
    
    Py_RETURN_TRUE;
}

// ==================== Method Definitions ====================

static PyMethodDef AEBFStream_methods[] = {
    {"process", (PyCFunction)AEBFStream_process, METH_VARARGS,
     "Process incoming bytes: process(data: bytes) -> int\n"
     "Returns number of bytes processed."},
    
    {"reset", (PyCFunction)AEBFStream_reset, METH_NOARGS,
     "Reset stream state (error recovery)"},
    
    {"clear", (PyCFunction)AEBFStream_clear, METH_NOARGS,
     "Clear buffer completely"},
    
    {"get_stats", (PyCFunction)AEBFStream_get_stats, METH_NOARGS,
     "Get statistics dictionary: {\n"
     "  'received': int, 'valid': int,\n"
     "  'corrupted': int, 'resyncs': int\n"
     "}"},
    
    {"get_buffer_info", (PyCFunction)AEBFStream_get_buffer_info, METH_NOARGS,
     "Get buffer info: (total_size, used_bytes)"},
    
    {"resize_buffer", (PyCFunction)AEBFStream_resize_buffer, METH_VARARGS,
     "Resize buffer: resize_buffer(new_size: int) -> bool"},
    
    {NULL, NULL, 0, NULL}
};

// ==================== Type Definition ====================

static PyTypeObject AEBFStreamType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name = "aebf.Stream",
    .tp_doc = "AEBF Stream Parser - Process serial data with callbacks\n\n"
              "Args:\n"
              "    buffer_size: int (default 256) - Size of internal buffer\n"
              "    on_frame: callable(device_id, service_id, payload_bytes) - Called when frame received\n"
              "    on_error: callable(error_code) - Called on frame errors\n",
    .tp_basicsize = sizeof(AEBFStreamObject),
    .tp_itemsize = 0,
    .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_new = PyType_GenericNew,
    .tp_init = (initproc)AEBFStream_init,
    .tp_dealloc = (destructor)AEBFStream_dealloc,
    .tp_methods = AEBFStream_methods,
};

// Deallocator - Frees C memory
static void AEBFStream_dealloc(AEBFStreamObject* self) {
    // Deinit C stream (just resets state)
    aebfStream_reset(&self->stream);
    // ===== C MEMORY MANAGEMENT =====
    // Free the buffer we allocated
    if (self->buffer) {
        free(self->buffer);
        self->buffer = NULL;
        self->buffer_size = 0;
    }
    
    // Release Python callbacks
    Py_XDECREF(self->py_on_frame);
    Py_XDECREF(self->py_on_error);
    
    Py_TYPE(self)->tp_free((PyObject*)self);
}


// ==================== Module Definition ====================

static PyMethodDef aebf_methods[] = {
    {"is_frame_complete", py_is_frame_complete, METH_VARARGS,
     "Check if buffer contains a complete frame"},
    
    {"encode", py_encode, METH_VARARGS,
     "Encode a frame: encode(device_id, service_id, payload) -> bytes"},
    
    {"decode", py_decode, METH_VARARGS,
     "Decode a frame: decode(frame) -> (device_id, service_id, payload)"},
    
    {NULL, NULL, 0, NULL}  // Sentinel
};

static struct PyModuleDef aebf_module = {
    PyModuleDef_HEAD_INIT,
    "aebf",
    "AEBF Protocol - Asynchronous Expandable Binary Frame",
    -1,
    aebf_methods
};

PyMODINIT_FUNC PyInit_aebf(void) {
    PyObject* m;
    
    // READY THE STREAM TYPE
    if (PyType_Ready(&AEBFStreamType) < 0)
        return NULL;
    
    // CREATE MODULE
    m = PyModule_Create(&aebf_module);
    if (m == NULL)
        return NULL;
    
    // ===== ADD STREAM CLASS TO MODULE =====
    Py_INCREF(&AEBFStreamType);
    if (PyModule_AddObject(m, "Stream", (PyObject*)&AEBFStreamType) < 0) {
        Py_DECREF(&AEBFStreamType);
        Py_DECREF(m);
        return NULL;
    }
    
    // Optional: Add version constants
    PyModule_AddIntConstant(m, "__version__", 1);
    PyModule_AddStringConstant(m, "__author__", "Your Name");
    
    return m;
}