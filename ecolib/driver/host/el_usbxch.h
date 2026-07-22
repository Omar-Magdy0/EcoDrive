#ifndef USBXCH
#define USBXCH
#include <stdint.h>
#include <stdbool.h>
#include "TCPServer.h"


typedef struct{
    TCPServer tcps;
}el_usbxch_handle_t;

void el_usbxch_init(el_usbxch_handle_t *h);
uint32_t el_usbxch_write(el_usbxch_handle_t *h, const uint8_t* data, uint32_t length);
uint32_t el_usbxch_write_available(el_usbxch_handle_t *h);
uint32_t el_usbxch_flush(el_usbxch_handle_t *h);
uint32_t el_usbxch_read(el_usbxch_handle_t *h, uint8_t *data, uint32_t length);
uint32_t el_usbxch_read_available(el_usbxch_handle_t *h);
bool el_usbxch_connected(el_usbxch_handle_t *h);
void el_usbxch_update(el_usbxch_handle_t *h);

#endif


