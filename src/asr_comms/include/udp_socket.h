#pragma once

#include "transport.h"

#include <string>
#include <stdexcept>
#include <cstring>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// Bidirectional UDP socket: binds a local port for receiving
// and targets a fixed remote address for sending.
class UdpSocket : public ITransport {
public:
    UdpSocket(uint16_t bind_port, const std::string& target_ip, uint16_t target_port)
    {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("socket() failed");

        sockaddr_in local{};
        local.sin_family      = AF_INET;
        local.sin_addr.s_addr = INADDR_ANY;
        local.sin_port        = htons(bind_port);
        if (bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
            close(fd_);
            throw std::runtime_error("bind() failed on port " + std::to_string(bind_port));
        }

        std::memset(&target_, 0, sizeof(target_));
        target_.sin_family = AF_INET;
        target_.sin_port   = htons(target_port);
        if (inet_pton(AF_INET, target_ip.c_str(), &target_.sin_addr) != 1) {
            close(fd_);
            throw std::runtime_error("Invalid target IP: " + target_ip);
        }
    }

    ~UdpSocket() { if (fd_ >= 0) close(fd_); }

    UdpSocket(const UdpSocket&)            = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    ssize_t recv(void* buf, size_t len) override
    {
        return ::recv(fd_, buf, len, 0);
    }

    ssize_t send(const void* buf, size_t len) override
    {
        return sendto(fd_, buf, len, 0,
                      reinterpret_cast<const sockaddr*>(&target_), sizeof(target_));
    }

private:
    int         fd_{-1};
    sockaddr_in target_{};
};
