
// TcpTransport.h
#pragma once
#include "ITransport.h"
#include <boost/asio.hpp>
#include <memory>
#include <thread>
#include <deque>
class TcpTransport : public ITransport {
public:
    TcpTransport(boost::asio::io_context& io, const std::string& host, uint16_t port);
    ~TcpTransport();

    void start_receive(DataCallback on_data) override;
    void send(const uint8_t* data, size_t len) override;
    bool is_connected() const override;

private:
    void do_read();
    void do_write();

    boost::asio::io_context& io_;
    boost::asio::ip::tcp::socket socket_;
    DataCallback on_data_;

    std::vector<uint8_t> read_buffer_;
    std::vector<uint8_t> write_buffer_;
    std::mutex write_mutex_;
    std::deque<std::vector<uint8_t>> write_queue_;
    bool write_in_progress_ = false;
};