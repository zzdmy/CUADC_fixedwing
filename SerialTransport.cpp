// SerialTransport.cpp
#include "SerialTransport.h"
#include "AppLogger.h"
#include <array>
#include <deque>
#include <vector>
#include <mutex>

SerialTransport::SerialTransport(boost::asio::io_context& io, const std::string& port, unsigned int baud)
    : io_(io), serial_(io) {
    serial_.open(port);
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    AppLogger::get().info("Serial opened: {}", port);
}

SerialTransport::~SerialTransport() {
    boost::system::error_code ec;
    serial_.close(ec);
}

void SerialTransport::start_receive(DataCallback on_data) {
    on_data_ = std::move(on_data);
    do_read();
}

void SerialTransport::do_read() {
    serial_.async_read_some(
        boost::asio::buffer(read_buf_),
        [this](const boost::system::error_code& ec, size_t n) {
            if (!ec && on_data_) {
                on_data_(read_buf_.data(), n);
                do_read();
            }
            // 可选：处理 error（如断开）
        });
}

void SerialTransport::send(const uint8_t* data, size_t len) {
    if (!is_connected()) return;

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

void SerialTransport::do_write() {
    if (write_queue_.empty()) {
        write_in_progress_ = false;
        return;
    }

    write_buffer_ = std::move(write_queue_.front());
    write_queue_.pop_front();

    boost::asio::async_write(
        serial_,
        boost::asio::buffer(write_buffer_),
        [this](const boost::system::error_code& ec, size_t /*n*/) {
            if (!ec) {
                do_write(); // 继续发送下一条
            }
            else {
                // 可选：记录错误
                write_in_progress_ = false;
            }
        });
}

bool SerialTransport::is_connected() const {
    return serial_.is_open();
}