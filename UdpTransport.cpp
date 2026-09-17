// UdpTransport.cpp
#include "UdpTransport.h"
#include "AppLogger.h"

UdpTransport::UdpTransport(boost::asio::io_context& io, const std::string& remote_host, uint16_t remote_port)
    : io_(io), socket_(io, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0)) {
    boost::asio::ip::udp::resolver resolver(io_);
    auto endpoints = resolver.resolve(remote_host, std::to_string(remote_port));
    remote_endpoint_ = *endpoints.begin();
    AppLogger::get().info("UDP ready to send to {}:{}", remote_host, remote_port);
    do_receive();
}

UdpTransport::~UdpTransport() {
    socket_.close();
}

void UdpTransport::start_receive(DataCallback on_data) {
    on_data_ = std::move(on_data);
    // already started in constructor
}

void UdpTransport::do_receive() {
    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_),
        sender_endpoint_,
        [this](const boost::system::error_code& ec, size_t n) {
            if (!ec && on_data_) {
                on_data_(recv_buffer_.data(), n);
            }
            do_receive(); // 继续接收
        });
}

void UdpTransport::send(const uint8_t* data, size_t len) {
    socket_.send_to(boost::asio::buffer(data, len), remote_endpoint_);
}

