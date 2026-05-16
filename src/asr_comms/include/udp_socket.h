#pragma once

#include "transport.h"

#include <atomic>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

class UdpSocket : public ITransport {
public:
    // Client mode: send target is known upfront.
    UdpSocket(uint16_t bind_port, const std::string& target_ip, uint16_t target_port)
        : peer_known_(true)
    {
        init(bind_port);
        std::memset(&target_, 0, sizeof(target_));
        target_.sin_family = AF_INET;
        target_.sin_port   = htons(target_port);
        if (inet_pton(AF_INET, target_ip.c_str(), &target_.sin_addr) != 1) {
            close(fd_);
            throw std::runtime_error("Invalid target IP: " + target_ip);
        }
    }

    // Server mode: send target is learned from the first incoming packet.
    explicit UdpSocket(uint16_t bind_port)
        : peer_known_(false)
    {
        init(bind_port);
        std::memset(&target_, 0, sizeof(target_));
    }

    ~UdpSocket() { if (fd_ >= 0) close(fd_); }
    UdpSocket(const UdpSocket&)            = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Set a receive timeout so recv threads can exit cleanly when running_ = false.
    void set_recv_timeout_ms(int ms)
    {
        timeval tv{};
        tv.tv_sec  = ms / 1000;
        tv.tv_usec = (ms % 1000) * 1000;
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    bool peer_known() const { return peer_known_.load(std::memory_order_acquire); }

    // Override the send target (used by UAV when a peer beacon arrives over SiK).
    void set_target(uint32_t ip_net_order, uint16_t port_host_order)
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_.sin_family      = AF_INET;
        target_.sin_addr.s_addr = ip_net_order;
        target_.sin_port        = htons(port_host_order);
        peer_known_.store(true, std::memory_order_release);
    }

    ssize_t recv(void* buf, size_t len) override
    {
        sockaddr_in from{};
        socklen_t   from_len = sizeof(from);
        const ssize_t n = recvfrom(fd_, buf, len, 0,
                                   reinterpret_cast<sockaddr*>(&from), &from_len);
        // In server mode, latch the sender address on first packet.
        if (n > 0 && !peer_known_.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (!peer_known_.load(std::memory_order_relaxed)) {
                target_ = from;
                peer_known_.store(true, std::memory_order_release);
            }
        }
        return n;
    }

    ssize_t send(const void* buf, size_t len) override
    {
        if (!peer_known_.load(std::memory_order_acquire)) return 0;
        std::lock_guard<std::mutex> lock(target_mutex_);
        return sendto(fd_, buf, len, 0,
                      reinterpret_cast<const sockaddr*>(&target_), sizeof(target_));
    }

private:
    int                fd_{-1};
    sockaddr_in        target_{};
    std::atomic<bool>  peer_known_;
    std::mutex         target_mutex_;

    void init(uint16_t bind_port)
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
    }
};
