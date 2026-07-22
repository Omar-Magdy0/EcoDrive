#include "el_usbxch.h"
#include <stdio.h>
#include <iostream>


void el_usbxch_init(el_usbxch_handle_t *h)
{
    bool success = h->tcps.init(4001);
    if(!success)
    {
        std::cerr << "USBXCH TCP init fail" << std::endl;
    }else
    {
        std::cout << "USBXCH TCP INITIALIZED" << std::endl;
    }
}

uint32_t el_usbxch_write(el_usbxch_handle_t *h, const uint8_t *data, uint32_t length)
{
    return h->tcps.write(data, length);
}

uint32_t el_usbxch_read(el_usbxch_handle_t *h, uint8_t *data, uint32_t length)
{
    return h->tcps.read(data, length);
}

bool el_usbxch_connected(el_usbxch_handle_t *h)
{
    return h->tcps.isConnected();
}

void el_usbxch_update(el_usbxch_handle_t *h)
{
    h->tcps.pollClient();
}

uint32_t el_usbxch_write_available(el_usbxch_handle_t *h)
{
    return UINT16_MAX;
}
uint32_t el_usbxch_flush(el_usbxch_handle_t *h)
{
    return 0;
}
uint32_t el_usbxch_read_available(el_usbxch_handle_t *h)
{
    return h->tcps.available();
}