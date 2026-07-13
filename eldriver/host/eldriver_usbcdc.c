#include "eldriver_usbcdc.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

#ifdef ELDRIVER_USBCDC_ENABLED

// Simple UART handle extension for file descriptor


// Initialize by opening the socat-created virtual serial port
void eldriver_usbcdc_init(eldriver_uart_handle_t *handle)
{
    if (!handle) return;

    // Open the virtual port
    int fd = open("/dev/ttyACM_CDC0", O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Failed to open /dev/ttyACM_CDC0");
        handle->fd = -1;
        return;
    }

    // Configure port: 115200 8N1 raw mode
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr failed");
        close(fd);
        handle->fd = -1;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;            // no parity
    tty.c_cflag &= ~CSTOPB;            // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                // 8 data bits
    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem lines, enable reading

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // no software flow control
    tty.c_oflag &= ~OPOST;                          // raw output

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        close(fd);
        handle->fd = -1;
        return;
    }

    handle->fd = fd;
}

// Write exactly 'len' bytes from data
uint8_t eldriver_usbcdc_write(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{
    if (!handle || handle->fd < 0 || !data || len == 0)
        return 0;

    ssize_t written = write(handle->fd, data, len);
    return len; // total bytes written
}

// Other functions left empty
uint16_t eldriver_usbcdc_read(eldriver_uart_handle_t *handle, uint8_t* data, uint8_t len)
{
    (void)handle; (void)data; (void)len;
    return 0;
}

elcore_rstream_stats_t eldriver_usbcdc_rx_stats(eldriver_uart_handle_t *handle)
{
    (void)handle;
    return (elcore_rstream_stats_t){0};
}

elcore_rstream_stats_t eldriver_usbcdc_tx_stats(eldriver_uart_handle_t *handle)
{
    (void)handle;
    return (elcore_rstream_stats_t){0};
}

void eldriver_usbcdc_resetStats(eldriver_uart_handle_t *handle)
{
    (void)handle;
}

#endif