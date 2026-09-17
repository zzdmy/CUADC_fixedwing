// mavlinkserver.h
#ifndef SERIALPORTHANDLER_H
#define SERIALPORTHANDLER_H

#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <windows.h>
#include <common/mavlink.h>   // MAVLink协议
#include <queue>              // 数据存储队列
#include <mutex>              // 共享资源同步锁
#include <condition_variable> // 线程间通信
#include "yolodecet/yolo_tracker.h"
#include "MissionScheduler.h"
#include <optional>//

#include <setupapi.h>
#include <devguid.h>
#include <regstr.h>

// 外部系统参数声明
extern int system_id;
extern int component_id;
extern uint8_t target_system;
extern uint8_t target_component;
extern std::atomic<float>distance_to_ground;//定义储存对地距离原子变量

/// 飞行模式数据结构
struct FlightModeData {
	uint8_t base_mode;     // 基础模式位掩码
	uint32_t custom_mode;  // 自定义模式值
};

/// 返航点位置数据结构
struct HomePositionData {
	double latitude;   // 纬度 (degE7)
	double longitude;  // 经度 (degE7)
	float altitude;    // 海拔高度 (mm)
};

#include <queue>
#include <mutex>
#include <condition_variable>
#include <utility>

template<typename T>
class SafeQueue {
private:
	mutable std::mutex mutex;
	std::queue<T> queue;
	std::condition_variable cond;
	bool closed = false;

public:
	SafeQueue() = default;
	~SafeQueue() = default;

	// 禁止拷贝，允许移动
	SafeQueue(const SafeQueue&) = delete;
	SafeQueue& operator=(const SafeQueue&) = delete;

	/**
	 * @brief 入队一个元素（支持移动语义）
	 */
	bool enqueue(T&& item) {
		std::lock_guard<std::mutex> lock(mutex);
		if (closed) {
			return false; // 已关闭，拒绝入队
		}
		queue.push(std::move(item));
		cond.notify_one();
		return true; // 入队成功
	}

	/**
	 * @brief 阻塞出队，直到有元素或队列关闭
	 * @return 成功返回元素，失败抛出异常（如已关闭且为空）
	 */
	T dequeue() {
		std::unique_lock<std::mutex> lock(mutex);
		cond.wait(lock, [this] { return !queue.empty() || closed; });

		if (queue.empty()) {
			throw std::runtime_error("SafeQueue: cannot dequeue, queue is closed.");
		}

		T item = std::move(queue.front());
		queue.pop();
		return item;
	}

	/**
	 * @brief 非阻塞尝试出队
	 * @param item 输出参数
	 * @return true 成功获取；false 队列为空
	 */
	bool try_dequeue(T& item) {
		std::lock_guard<std::mutex> lock(mutex);
		if (queue.empty()) return false;
		item = std::move(queue.front());
		queue.pop();
		return true;
	}

	/**
	 * @brief 带超时的出队
	 * @param item 输出参数
	 * @param timeout_ms 超时时间（毫秒）
	 * @return true 成功；false 超时或关闭
	 */
	bool dequeue_for(T& item, int timeout_ms) {
		std::unique_lock<std::mutex> lock(mutex);
		auto timeout = std::chrono::milliseconds(timeout_ms);
		bool ready = cond.wait_for(lock, timeout, [this] { return !queue.empty() || closed; });

		if (!ready || queue.empty()) return false;

		item = std::move(queue.front());
		queue.pop();
		return true;
	}

	/**
	 * @brief 检查是否为空
	 */
	bool empty() const {
		std::lock_guard<std::mutex> lock(mutex);
		return queue.empty();
	}

	/**
	 * @brief 获取当前大小
	 */
	size_t size() const {
		std::lock_guard<std::mutex> lock(mutex);
		return queue.size();
	}

	/**
	 * @brief 关闭队列：通知所有等待线程退出
	 */
	void close() {
		std::lock_guard<std::mutex> lock(mutex);
		closed = true;
		cond.notify_all();
	}

	/**
	 * @brief 检查队列是否已关闭
	 */
	bool is_closed() const {
		std::lock_guard<std::mutex> lock(mutex);
		return closed;
	}

};

/// MAVLink通信处理器（支持串口/TCP/UDP）
class SerialPortHandler {
public:
	
	// 构造函数组
	SerialPortHandler(boost::asio::io_context& io, const std::string& port, unsigned int baud_rate);  // 串口
	SerialPortHandler(boost::asio::io_context& io, const std::string& host, unsigned short port);     // TCP
	SerialPortHandler(boost::asio::io_context& io, const std::string& host, unsigned short port, bool is_udp); // UDP

	~SerialPortHandler();  // 析构函数

	// 核心通信方法
	void start_receive();  // 启动数据接收
	void send_mavlink_message(const mavlink_message_t& msg);  // 发送MAVLink消息

	// 数据获取接口
	bool try_get_mission_ack_data(mavlink_mission_ack_t& mission_ack_data);
	bool try_get_mission_request_int_data(mavlink_mission_request_int_t& request_data);
	bool try_get_mission_request_data(mavlink_mission_request_t& request_data);
	bool try_get_gps_data(mavlink_global_position_int_t& gps_data);
	bool try_get_gps_raw_data(mavlink_gps_raw_int_t& gps_raw_data);
	bool try_get_imu_data(mavlink_attitude_t& imu_data);
	bool try_get_rc_channels(mavlink_rc_channels_t& rc_channels);
	bool try_get_servo_pwm(uint8_t servo_index, uint16_t& pwm);
	bool try_get_flight_mode(FlightModeData& mode);
	bool try_get_home_position(HomePositionData& home_pos);
	bool try_get_distance_sensor_data(mavlink_distance_sensor_t& distance_sensor_data);

	// 任务管理接口
	static bool try_get_latest_mission_current(mavlink_mission_current_t& current);
	static void clear_mission_current();  // 清空任务状态

	// 飞行控制指令
	void setup_guided_position_local(mavlink_message_t& msg, float x, float y, float z, float yaw);//基于本地坐标系位置
	void setup_guided_position_gps(mavlink_message_t& msg, double  lat_deg, double  lon_deg, float  alt_amsl, float yaw);// 基于GPS坐标系位置
	void setup_guided_position_body(mavlink_message_t& msg, float x, float y, float z, float yaw);// 基于机体坐标系位置
	bool setFlightMode(uint8_t flight_mode);
	void setup_guided_velocity(mavlink_message_t& msg, float vx, float vy, float vz, float yaw_rate);// 基于机体坐标系速度
	void setup_flight_mode_message(mavlink_message_t& msg, uint8_t flight_mode);
	void setup_pwm_message(mavlink_message_t& msg, uint16_t pwm_value, uint16_t servo_n);

	// 消息队列管理
	void enqueueMessage(mavlink_message_t&& msg);

	

private:
	std::atomic<size_t> total_bytes_received_{ 0 };  // 累计接收字节数（线程安全）
	// 任务状态管理
	static mavlink_mission_current_t latest_mission_current_;
	static std::atomic<bool> has_new_mission_current_;
	static std::mutex mission_current_mutex_;

	// 连接类型标识
	enum class ConnectionType { Serial, TCP, UDP };
	ConnectionType connection_type_;

	// 通信核心方法
	void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
	void parse_mavlink_message(uint8_t chan, const uint8_t* buffer, size_t len);
	void sendMessageLoop();

	// 通信对象
	boost::asio::serial_port serial_;                                // 串口对象
	std::shared_ptr<boost::asio::ip::tcp::socket> tcp_socket_;       // TCP套接字
	std::shared_ptr<boost::asio::ip::udp::socket> udp_socket_;       // UDP套接字
	boost::asio::ip::udp::endpoint remote_endpoint_;                 // UDP端点
	std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> read_buffer_;        // 读缓冲区

	// 数据存储
	std::optional<mavlink_mission_ack_t> latest_mission_ack_;
	std::optional<mavlink_mission_request_int_t> latest_mission_request_int_;
	std::optional<mavlink_mission_request_t> latest_mission_request_;
	std::optional<mavlink_global_position_int_t> latest_gps_data_;
	std::optional<mavlink_gps_raw_int_t> latest_gps_raw_data_;
	std::optional<mavlink_attitude_t> latest_imu_data_;
	std::optional<mavlink_rc_channels_t> latest_rc_channels_;
	std::optional<mavlink_servo_output_raw_t> latest_servo_output_;
	std::optional<FlightModeData> latest_flight_mode_;
	std::optional<HomePositionData> latest_home_position_;
	std::optional<mavlink_distance_sensor_t> latest_distance_sensor_;

	// 同步控制
	std::mutex mission_ack_data_mutex_;
	std::mutex mission_request_int_data_mutex_;
	std::mutex mission_request_data_mutex_;
	std::mutex gps_data_mutex_, imu_data_mutex_, rc_channels_mutex_;
	std::mutex distance_sensor_data_mutex_, gps_raw_data_mutex_;
	std::mutex servo_output_mutex_;
	std::mutex flight_mode_mutex_;
	std::mutex home_position_mutex_;
	std::condition_variable data_cv_;

	// 消息处理线程
	SafeQueue<mavlink_message_t> messageQueue;
	std::thread senderThread;
	std::atomic<bool> running{ true };
};

#endif // SERIALPORTHANDLER_H