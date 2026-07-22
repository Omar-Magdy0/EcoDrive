#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>

// TODO #22: Add a Cleaner Asio TCP Client for EcoTool
class TcpClient
{
public:
    TcpClient() = default;

    ~TcpClient()
    {
        disconnect();
    }

    bool connect(const std::string& host, uint16_t port)
    {
        disconnect();

        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0)
            return false;

        int buffer_size = 4 * 1024 * 1024;
        setsockopt(sock_, SOL_SOCKET, SO_SNDBUF,
                   &buffer_size, sizeof(buffer_size));
        setsockopt(sock_, SOL_SOCKET, SO_RCVBUF,
                   &buffer_size, sizeof(buffer_size));
                   
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);

        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1)
        {
            disconnect();
            return false;
        }

        if (::connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0)
        {
            disconnect();
            return false;
        }

        int flags = fcntl(sock_, F_GETFL, 0);
        fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

        return true;
    }

    void disconnect()
    {
        if (sock_ >= 0)
        {
            close(sock_);
            sock_ = -1;
        }
    }

    bool is_connected() const
    {
        return sock_ >= 0;
    }

    int available() const
    {
        if (sock_ < 0)
            return -1;

        int bytes = 0;

        if (ioctl(sock_, FIONREAD, &bytes) < 0)
            return -1;

        return bytes;
    }

    int write(const void* data, size_t length)
    {
        if (sock_ < 0)
            return -1;

        return send(sock_, data, length, MSG_NOSIGNAL);
    }

    int read(void* buffer, size_t length)
    {
        if (sock_ < 0)
            return -1;

        return recv(sock_, buffer, length, 0);
    }

private:
    int sock_ = -1;
};