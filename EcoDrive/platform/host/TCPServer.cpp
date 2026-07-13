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

bool TCPServer::init(uint16_t port)
{
    stop();

    listen_sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock_ < 0)
        return false;

    int opt = 1;
    setsockopt(listen_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int buffer_size = 4 * 1024 * 1024;

    setsockopt(listen_sock_, SOL_SOCKET, SO_SNDBUF,
               &buffer_size, sizeof(buffer_size));
    
    setsockopt(listen_sock_, SOL_SOCKET, SO_RCVBUF,
               &buffer_size, sizeof(buffer_size));


    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(listen_sock_, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        stop();
        return false;
    }

    if (listen(listen_sock_, 1) < 0)
    {
        stop();
        return false;
    }

    // Make the listening socket non-blocking
    int flags = fcntl(listen_sock_, F_GETFL, 0);
    fcntl(listen_sock_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

bool TCPServer::pollClient()
{
    if (client_sock_ >= 0)
        return true;

    client_sock_ = accept(listen_sock_, nullptr, nullptr);

    if (client_sock_ < 0)
        return false;

    int flags = fcntl(client_sock_, F_GETFL, 0);
    fcntl(client_sock_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

void TCPServer::disconnect()
{
    if (client_sock_ >= 0)
    {
        close(client_sock_);
        client_sock_ = -1;
    }
}

void TCPServer::stop()
{
    disconnect();

    if (listen_sock_ >= 0)
    {
        close(listen_sock_);
        listen_sock_ = -1;
    }
}

bool TCPServer::isConnected() const
{
    return client_sock_ >= 0;
}

int TCPServer::available() const
{
    if (client_sock_ < 0)
        return -1;

    int bytes = 0;

    if (ioctl(client_sock_, FIONREAD, &bytes) < 0)
        return -1;

    return bytes;
}

int TCPServer::write(const void* data, size_t len)
{
    if (client_sock_ < 0)
        return -1;

    return send(client_sock_, data, len, MSG_NOSIGNAL);
}

int TCPServer::read(void* data, size_t len)
{
    if (client_sock_ < 0)
        return 0;
    int32_t rec = recv(client_sock_, data, len, 0);
    return ((rec >= 0)?rec:0);
}