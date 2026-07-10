#ifndef USBXCH
#define USBXCH
#include <stdint.h>
#include <stdbool.h>
#include "elcore_rstream.h"
#include "eldriver_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    uint8_t itf;
    uint16_t tx_min_freespace;
    uint16_t tx_blocks;
    uint16_t rx_high_watermark;
}eldriver_usbxch_handle_t;

void eldriver_usbxch_init(eldriver_usbxch_handle_t *h);
uint32_t eldriver_usbxch_write(eldriver_usbxch_handle_t *h, const uint8_t* data, uint32_t length);
uint32_t eldriver_usbxch_write_available(eldriver_usbxch_handle_t *h);
uint32_t eldriver_usbxch_flush(eldriver_usbxch_handle_t *h);
uint32_t eldriver_usbxch_clear(eldriver_usbxch_handle_t *h);
uint32_t eldriver_usbxch_read(eldriver_usbxch_handle_t *h, uint8_t *data, uint32_t length);
uint32_t eldriver_usbxch_read_available(eldriver_usbxch_handle_t *h);
bool eldriver_usbxch_connected(eldriver_usbxch_handle_t *h);
void eldriver_usbxch_update(eldriver_usbxch_handle_t *h);

#ifdef __cplusplus
}
#endif
#endif