#include "sys.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "aebfStream.h"
#include <array>
#include "eldriver_uart.h"

constexpr uint8_t AEBF_PWM_SERVICEID = 0xC1;
//==================== FREERTOS Variables ===============================
constexpr size_t AEBF_ENCODE_BUFFER_SIZE = 255;
std::array<uint8_t, AEBF_ENCODE_BUFFER_SIZE> aebf_encode_buffer{};

SemaphoreHandle_t aebf_mutex;
TaskHandle_t xAebf_handle;

constexpr uint8_t AEBF_PRIO1_QUEUE_LENGTH = 20;
constexpr uint8_t LOG_MAX_SIZE = 64;

#include "eldriver/eldriver_usbcdc.h"

eldriver_uart_handle_t serial;

#define serial_init() eldriver_usbcdc_init(&serial)
#define serial_write(data, len) eldriver_usbcdc_write(&serial, data, len)
#define serial_read(data, len) eldriver_usbcdc_read(&serial, len)
#define serial_rx_stats() eldriver_usbcdc_rx_stats(&serial)
#define serial_tx_stats() eldriver_usbcdc_tx_stats(&serial)

#include "eldriver/eldriver_core.h"
eldriver_core_t core;

QueueHandle_t aebf_queue_prio1;

enum class AebfMsgType : uint8_t {
    Log = 0,
    Debug,
    ModbusRtu
};

struct aebf_message_t {
    uint8_t type{};
    uint8_t len{};
    uint8_t buffer[LOG_MAX_SIZE]{};
};

//========================================================================

pwmDataBuffer_t pwmDataBuffer;

uint32_t logMessages = 0;
uint32_t buffer_overrun = 0;

void aebf_print_string_prio1(char *data, uint8_t len, AebfMsgType type, uint8_t timeout_ms)
{
    aebf_message_t msg;
    msg.type = static_cast<uint8_t>(type);
    msg.len = len;
    if(len < LOG_MAX_SIZE) memcpy(msg.buffer, data, len);
    xQueueSend(aebf_queue_prio1, &msg, pdMS_TO_TICKS(timeout_ms));
    xTaskNotifyGive(xAebf_handle);
}

#include <stdarg.h>
void print_log(const char* format, ...)
{
    char buffer[LOG_MAX_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, LOG_MAX_SIZE, format, args);
    va_end(args);
    if(len > 0){
        aebf_print_string_prio1(buffer, (len < LOG_MAX_SIZE) ? len : LOG_MAX_SIZE-1, AebfMsgType::Log, 0);
    }
}

void print_debug(const char* format, ...)
{
    char buffer[LOG_MAX_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, LOG_MAX_SIZE, format, args);
    va_end(args);
    if(len > 0){
        aebf_print_string_prio1(buffer, (len < LOG_MAX_SIZE) ? len : LOG_MAX_SIZE-1, AebfMsgType::Debug, 0);
    }
}

void dummyTask_log(void* argument){
    while(1){
        print_log("Hello %lu", logMessages++);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#define HOST

#ifndef HOST
extern elcore_rstream_t cdc_tx_buffer; // Assume this is defined and initialized elsewhere
#endif 

void dummyTask_debug(void* argument){
    // Previous counters for throughput calculation
    static uint32_t last_bytes_written = 0;
    static uint32_t last_bytes_read    = 0;
    static TickType_t last_tick        = 0;
    static uint8_t tx_wm_counter       = 0;
    while (1) {
        #ifndef HOST
        TickType_t now_tick = xTaskGetTickCount();
        float elapsed_s = (now_tick - last_tick) / (float)configTICK_RATE_HZ;

        elcore_rstream_stats_t stats = elcore_rstream_getStats(&cdc_tx_buffer);
        // --- Compute throughput (bytes/sec) ---
        uint32_t write_rate = (stats.bytes_written - last_bytes_written) / elapsed_s;
        uint32_t read_rate  = (stats.bytes_read    - last_bytes_read)    / elapsed_s;

        last_bytes_written = stats.bytes_written;
        last_bytes_read    = stats.bytes_read;
        last_tick = now_tick;

        // --- RTOS health metrics ---
        uint32_t free_heap = xPortGetFreeHeapSize();              // Free heap
        UBaseType_t task_count = uxTaskGetNumberOfTasks();       // Optional: number of tasks
        // Optional: iterate tasks to check states, stack high-water marks, etc.

        // --- Debug prints ---
        print_debug("SER_TX: WR:%lu, RD:%lu, FW:%lu, HW:%u", write_rate, read_rate, stats.failed_writes, stats.high_watermark);
        tx_wm_counter++;
        if(tx_wm_counter >= 20)
        {
            tx_wm_counter = 0;
            elcore_rstream_resetStats(&cdc_tx_buffer); // Reset stats periodically to prevent overflow and get fresh rates
        }
        #endif
        // --- Optional: additional metrics ---
        //print_debug("Task Stack HighWater: %u", uxTaskGetStackHighWaterMark(AEBF_Handle));
        //print_debug("Queue Lengths, Semaphore Counts, etc.");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void xAEBF(void* argument)
{
    while(1)
    {
        aebf_message_t msg;

        if(xQueueReceive(aebf_queue_prio1, &msg, 0) == pdPASS)
        {
            // Process message
            switch (msg.type)
            {
            case static_cast<uint8_t>(AebfMsgType::Log):
                // Handle log message (e.g., print to console)
                memccpy(aebf_encode_buffer.data() + 5, msg.buffer, 0, msg.len);
                aebf_encode_frame(aebf_encode_buffer.data(), AEBF_DEVICE_ID, AEBF_LOG_SERVICEID, msg.len);
                break;
            case static_cast<uint8_t>(AebfMsgType::Debug):
                // Handle debug message
                memccpy(aebf_encode_buffer.data() + 5, msg.buffer, 0, msg.len);
                aebf_encode_frame(aebf_encode_buffer.data(), AEBF_DEVICE_ID, AEBF_DEBUG_SERVICEID, msg.len);
                break;

            default:
                break;
            }
            serial_write(aebf_encode_buffer.data(), AEBF_FRAMED_SIZE(msg.len));
            continue;
        }

        // No messages, check for prio0 aebf_tasks (e.g., PWM stream)
        PwmDataFrame_t *frame;
        if(pwmDataBuffer.readFrame(&frame))
        {
            // Process a PWM data frame 
            memcpy(aebf_encode_buffer.data() + 5, frame, sizeof(PwmDataFrame_t));
            aebf_encode_frame(aebf_encode_buffer.data(), AEBF_DEVICE_ID, AEBF_PWM_SERVICEID, sizeof(PwmDataFrame_t));
            serial_write(aebf_encode_buffer.data(), AEBF_FRAMED_SIZE(sizeof(PwmDataFrame_t)));
            continue;
        }

        // If we get here then all messages are processed, wait for notification (either from prioN messages or new PWM data)
        ulTaskNotifyTake(pdTRUE,     // Clear notification count on exit
                         1);  // Wait 4ms        
    }
}

void sys_init()
{
    platform_init();
    eldriver_core_init(&core);
    pwmDataBuffer.init();
    serial.config.baudrate = 912600;
    serial_init();
    //enable motor control function

    aebf_mutex = xSemaphoreCreateMutex();
    xSemaphoreGive(aebf_mutex);  // Make it available initially
    aebf_queue_prio1 = xQueueCreate(AEBF_PRIO1_QUEUE_LENGTH, sizeof(aebf_message_t));

    xTaskCreate(
        xAEBF,             // Task function
        "AEBF_task",            // Name (debug only)
        512,                    // Stack words (safe for now)
        NULL,                // Parameter
        2,                      // Priority (LOW)
        &xAebf_handle
    );

    xTaskCreate(
        dummyTask_log,             // Task function
        "dummyTask_log",            // Name (debug only)
        256,                    // Stack words (safe for now)
        NULL,                // Parameter
        1,                      // Priority (LOW)
        NULL
    );

    xTaskCreate(
        dummyTask_debug,             // Task function
        "dummyTask_debug",            // Name (debug only)
        256,                    // Stack words (safe for now)
        NULL,                // Parameter
        1,                      // Priority (LOW)
        NULL
    );

    motor_c1.init();
    #ifndef PLATFORM_HOST
    vTaskStartScheduler();
    #else
    freeRtos_init();
    gui_loop();
    #endif
}


