// TcpTransport.cpp
#include "TcpTransport.h"
#include "AppLogger.h"

TcpTransport::TcpTransport(boost::asio::io_context& io, const std::string& host, uint16_t port)
    : io_(io), socket_(io), read_buffer_(1024) {
    boost::asio::ip::tcp::resolver resolver(io_);
    auto endpoints = resolver.resolve(host, std::to_string(port));
    boost::asio::connect(socket_, endpoints);
    AppLogger::get().info("TCP connected to {}:{}", host, port);
}

TcpTransport::~TcpTransport() {
    boost::system::error_code ec;
    socket_.close(ec);
}

void TcpTransport::start_receive(DataCallback on_data) {
    on_data_ = std::move(on_data);
    do_read();
}

void TcpTransport::do_read() {
    socket_.async_read_some(
        boost::asio::buffer(read_buffer_),
        [this](const boost::system::error_code& ec, size_t n) {
            if (!ec && on_data_) {
                on_data_(read_buffer_.data(), n);
                do_read();
            }
            else {
                AppLogger::get().error("TCP read error: {}", ec.message());
            }
        });
}

void TcpTransport::send(const uint8_t* data, size_t len) {
    std::vector<uint8_t> copy(data, data + len);
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_queue_.push_back(std::move(copy));
    }
    if (!write_in_progress_) {
        write_in_progress_ = true;
        do_write();
    }
}

void TcpTransport::do_write() {
    if (write_queue_.empty()) {
        write_in_progress_ = false;
        return;
    }

    write_buffer_ = std::move(write_queue_.front());
    write_queue_.pop_front();

    boost::asio::async_write(
        socket_,
        boost::asio::buffer(write_buffer_),
        [this](const boost::system::error_code& ec, size_t /*n*/) {
            if (ec) {
                AppLogger::get().error("TCP write error: {}", ec.message());
                return;
            }
            do_write(); // 继续发送队列中下一个
        });
}

bool TcpTransport::is_connected() const {
    return socket_.is_open();
}