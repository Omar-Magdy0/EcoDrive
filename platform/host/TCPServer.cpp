#include "TCPServer.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <cstring>

static bool setNonBlocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0)
        return false;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) >= 0;
}

bool TCPServer::init(uint16_t port)
{
    stop();

    listen_sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock_ < 0)
        return false;

    int opt = 1;
    ::setsockopt(listen_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (::bind(listen_sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(listen_sock_);
        listen_sock_ = -1;
        return false;
    }

    if (::listen(listen_sock_, 1) < 0) {
        ::close(listen_sock_);
        listen_sock_ = -1;
        return false;
    }

    if (!setNonBlocking(listen_sock_)) {
        ::close(listen_sock_);
        listen_sock_ = -1;
        return false;
    }

    return true;
}

bool TCPServer::waitForClient()
{
    if (listen_sock_ < 0)
        return false;

    if (client_sock_ >= 0)
        return true;

    struct pollfd pfd;
    pfd.fd = listen_sock_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    int rc = ::poll(&pfd, 1, 0);
    if (rc <= 0)
        return false;

    if (pfd.revents & POLLIN) {
        sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        int client_fd = ::accept(listen_sock_, reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
        if (client_fd < 0)
            return false;

        if (!setNonBlocking(client_fd)) {
            ::close(client_fd);
            return false;
        }

        client_sock_ = client_fd;
        return true;
    }

    return false;
}

bool TCPServer::write(const uint8_t *bytes, uint32_t len)
{
    if (client_sock_ < 0 || !bytes || len == 0)
        return false;

    const uint8_t *ptr = bytes;
    uint32_t remaining = len;

    while (remaining > 0) {
        ssize_t sent = ::send(client_sock_, ptr, remaining, MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return false;
            disconnect();
            return false;
        }

        if (sent == 0)
            return false;

        ptr += sent;
        remaining -= static_cast<uint32_t>(sent);
    }

    return true;
}

uint32_t TCPServer::read(uint8_t *bytes, uint32_t len)
{
    if (client_sock_ < 0 || !bytes || len == 0)
        return 0;

    ssize_t received = ::recv(client_sock_, bytes, len, 0);
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            return 0;
        disconnect();
        return 0;
    }

    if (received == 0) {
        disconnect();
        return 0;
    }

    return static_cast<uint32_t>(received);
}

void TCPServer::stop()
{
    disconnect();
    if (listen_sock_ >= 0) {
        ::close(listen_sock_);
        listen_sock_ = -1;
    }
}

bool TCPServer::isConnected() const
{
    return client_sock_ >= 0;
}

void TCPServer::disconnect()
{
    if (client_sock_ >= 0) {
        ::shutdown(client_sock_, SHUT_RDWR);
        ::close(client_sock_);
        client_sock_ = -1;
    }
}
