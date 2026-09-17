// SerialTransport.h
#pragma once
#include "ITransport.h"
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <deque>
#include <mutex>

class SerialTransport : public ITransport {
public:
    SerialTransport(boost::asio::io_context& io, const std::string& port, unsigned int baud);
    ~SerialTransport() override;

    void start_receive(DataCallback on_data) override;
    void send(const uint8_t* data, size_t len) override;
    bool is_connected() const override;

private:
    void do_read();
    void do_write();

    boost::asio::io_context& io_;
    boost::asio::serial_port serial_;
    DataCallback on_data_;

    std::array<uint8_t, 1024> read_buf_;

    // 异步发送所需
    std::vector<uint8_t> write_buffer_;
    std::deque<std::vector<uint8_t>> write_queue_;
    std::mutex write_mutex_;
    bool write_in_progress_ = false;
};