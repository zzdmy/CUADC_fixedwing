// UdpTransport.h
#pragma once
#include "ITransport.h"
#include <boost/asio.hpp>
#include <memory>

class UdpTransport : public ITransport {
public:
    UdpTransport(boost::asio::io_context& io, const std::string& remote_host, uint16_t remote_port);
    ~UdpTransport();

    void start_receive(DataCallback on_data) override;
    void send(const uint8_t* data, size_t len) override;
    bool is_connected() const override { return true; } // UDP 无连接状态

private:
    void do_receive();

    boost::asio::io_context& io_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::asio::ip::udp::endpoint sender_endpoint_;
    DataCallback on_data_;
    std::array<uint8_t, 1024> recv_buffer_;
};