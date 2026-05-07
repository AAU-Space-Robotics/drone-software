#pragma once

#include "transport.h"

#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class SerialPort : public ITransport {
public:
    SerialPort(const std::string& device, int baud_rate)
    {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("Failed to open " + device);

        // Switch back to blocking after open to avoid ENXIO on some drivers
        int flags = fcntl(fd_, F_GETFL, 0);
        fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            close(fd_);
            throw std::runtime_error("tcgetattr failed on " + device);
        }

        const speed_t speed = to_speed(baud_rate);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        cfmakeraw(&tty);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;  // 100 ms read timeout — lets recv_loop check running_ regularly

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            close(fd_);
            throw std::runtime_error("tcsetattr failed on " + device);
        }
    }

    ~SerialPort() { if (fd_ >= 0) close(fd_); }
    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    ssize_t recv(void* buf, size_t len) override { return ::read(fd_, buf, len); }
    ssize_t send(const void* buf, size_t len) override { return ::write(fd_, buf, len); }

private:
    int fd_{-1};

    static speed_t to_speed(int baud)
    {
        switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud));
        }
    }
};
