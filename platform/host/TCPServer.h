#pragma once
#include <stdint.h>


class TCPServer
{
public:
    bool init(uint16_t port);
    bool waitForClient();
    bool write(const uint8_t *bytes, uint32_t len);
    uint32_t read(uint8_t *bytes, uint32_t len);
    void stop();
    bool isConnected() const;
    void disconnect();
private:
    int listen_sock_ = -1;
    int client_sock_ = -1;
};