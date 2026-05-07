#pragma once

#include <cstddef>
#include <sys/types.h>

struct ITransport {
    virtual ~ITransport() = default;
    virtual ssize_t recv(void* buf, size_t len) = 0;
    virtual ssize_t send(const void* buf, size_t len) = 0;
};
