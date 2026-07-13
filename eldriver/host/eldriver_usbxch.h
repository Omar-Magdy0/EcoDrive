#ifndef USBXCH
#define USBXCH
#include <stdint.h>
#include <stdbool.h>
#include "TCPServer.h"


typedef struct{
    TCPServer tcps;
}eldriver_usbxch_handle_t;

void eldriver_usbxch_init(eldriver_usbxch_handle_t *h);
uint32_t eldriver_usbxch_write(eldriver_usbxch_handle_t *h, const uint8_t* data, uint32_t length);
uint32_t eldriver_usbxch_write_available(eldriver_usbxch_handle_t *h);
uint32_t eldriver_usbxch_flush(eldriver_usbxch_handle_t *h);
uint32_t eldriver_usbxch_read(eldriver_usbxch_handle_t *h, uint8_t *data, uint32_t length);
uint32_t eldriver_usbxch_read_available(eldriver_usbxch_handle_t *h);
bool eldriver_usbxch_connected(eldriver_usbxch_handle_t *h);
void eldriver_usbxch_update(eldriver_usbxch_handle_t *h);

#endif


