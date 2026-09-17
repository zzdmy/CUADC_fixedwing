// ITransport.h
#pragma once
#include <functional>
#include <memory>
#include <cstddef>

class ITransport {
public:
    using DataCallback = std::function<void(const uint8_t*, size_t)>;

    virtual ~ITransport() = default;
    virtual void start_receive(DataCallback on_data) = 0;
    virtual void send(const uint8_t* data, size_t len) = 0;
    virtual bool is_connected() const = 0;
};