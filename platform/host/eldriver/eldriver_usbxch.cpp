#include "eldriver_usbxch.h"
#include <stdio.h>
#include <iostream>


void eldriver_usbxch_init(eldriver_usbxch_handle_t *h)
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

uint32_t eldriver_usbxch_write(eldriver_usbxch_handle_t *h, const uint8_t *data, uint32_t length)
{
    return h->tcps.write(data, length);
}

uint32_t eldriver_usbxch_read(eldriver_usbxch_handle_t *h, uint8_t *data, uint32_t length)
{
    return h->tcps.read(data, length);
}

bool eldriver_usbxch_connected(eldriver_usbxch_handle_t *h)
{
    return h->tcps.isConnected();
}

void eldriver_usbxch_update(eldriver_usbxch_handle_t *h)
{
    h->tcps.waitForClient();
}

