#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
class TCPServer
{
public:
    bool init(uint16_t port);
    bool pollClient();

    void disconnect();
    void stop();

    bool isConnected() const;

    int available() const;

    int write(const void* data, size_t len);

    int read(void* data, size_t len);

private:
    int listen_sock_ = -1;
    int client_sock_ = -1;
};