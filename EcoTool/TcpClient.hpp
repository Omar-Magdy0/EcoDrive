#pragma once

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <sys/ioctl.h>

/**
 * @brief Simple non-blocking TCP client implementation
 * 
 * Provides basic TCP operations with non-blocking API:
 * - connect/disconnect
 * - non-blocking read/write
 * - connection status check
 */
class TcpClient
{
public:
    /**
     * @brief Constructor - initializes socket to invalid state
     */
    TcpClient() : m_socket(-1) {}

    /**
     * @brief Destructor - ensures socket is closed
     */
    ~TcpClient()
    {
        disconnect();
    }

    /**
     * @brief Connect to remote TCP server
     * 
     * @param host IP address or hostname (e.g., "127.0.0.1" or "localhost")
     * @param port Port number (1-65535)
     * @return true on successful connection, false on failure
     */
    bool connect(const std::string& host, uint16_t port)
    {
        if (is_connected())
        {
            disconnect();
        }

        // Create socket
        m_socket = ::socket(AF_INET, SOCK_STREAM, 0);
        if (m_socket < 0)
        {
            return false;
        }

        // Set non-blocking mode
        if (!set_non_blocking(true))
        {
            ::close(m_socket);
            m_socket = -1;
            return false;
        }

        // Prepare server address
        struct sockaddr_in server_addr;
        std::memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);

        // Convert IP address
        if (::inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0)
        {
            ::close(m_socket);
            m_socket = -1;
            return false;
        }

        // Attempt connection (will return EINPROGRESS for non-blocking)
        if (::connect(m_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
        {
            if (errno != EINPROGRESS)
            {
                ::close(m_socket);
                m_socket = -1;
                return false;
            }
        }

        return true;
    }

    /**
     * @brief Disconnect from remote server
     */
    void disconnect()
    {
        if (m_socket >= 0)
        {
            ::close(m_socket);
            m_socket = -1;
        }
    }

    /**
     * @brief Check if currently connected to server
     * 
     * @return true if socket is valid and connected
     */
    bool is_connected() const
    {
        if (m_socket < 0)
        {
            return false;
        }

        // Check if socket is still valid
        int error = 0;
        socklen_t len = sizeof(error);
        if (::getsockopt(m_socket, SOL_SOCKET, SO_ERROR, &error, &len) < 0)
        {
            return false;
        }

        return error == 0;
    }

    /**
     * @brief Get number of bytes available to read (non-blocking)
     * 
     * @return Number of bytes available in read buffer, or -1 on error
     */
    int available() const
    {
        if (m_socket < 0)
        {
            return -1;
        }

        int bytes_available = 0;
        if (::ioctl(m_socket, FIONREAD, &bytes_available) < 0)
        {
            return -1;
        }

        return bytes_available;
    }

    /**
     * @brief Write data to socket (non-blocking)
     * 
     * @param data Pointer to data buffer
     * @param length Number of bytes to write
     * @return Number of bytes written, or -1 on error (check errno for EAGAIN/EWOULDBLOCK)
     */
    int write(const void* data, size_t length)
    {
        if (m_socket < 0)
        {
            errno = EBADF;
            return -1;
        }

        ssize_t bytes_sent = ::send(m_socket, data, length, MSG_NOSIGNAL);
        return static_cast<int>(bytes_sent);
    }

    /**
     * @brief Read data from socket (non-blocking)
     * 
     * @param buffer Pointer to receive buffer
     * @param length Maximum number of bytes to read
     * @return Number of bytes read, 0 if connection closed, or -1 on error 
     *         (check errno for EAGAIN/EWOULDBLOCK meaning no data available)
     */
    int read(void* buffer, size_t length)
    {
        if (m_socket < 0)
        {
            errno = EBADF;
            return -1;
        }

        ssize_t bytes_read = ::recv(m_socket, buffer, length, 0);
        return static_cast<int>(bytes_read);
    }

    /**
     * @brief Get the underlying socket file descriptor
     * 
     * @return Socket FD (-1 if not connected)
     */
    int get_socket() const
    {
        return m_socket;
    }

private:
    int m_socket;

    /**
     * @brief Set socket to blocking or non-blocking mode
     * 
     * @param non_blocking true for non-blocking, false for blocking
     * @return true on success
     */
    bool set_non_blocking(bool non_blocking)
    {
        if (m_socket < 0)
        {
            return false;
        }

        int flags = ::fcntl(m_socket, F_GETFL, 0);
        if (flags < 0)
        {
            return false;
        }

        if (non_blocking)
        {
            flags |= O_NONBLOCK;
        }
        else
        {
            flags &= ~O_NONBLOCK;
        }

        return ::fcntl(m_socket, F_SETFL, flags) >= 0;
    }
};
