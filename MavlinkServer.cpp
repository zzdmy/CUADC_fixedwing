// MavlinkServer.cpp
#include "MavlinkServer.h"
#include <iomanip>  // 用于 std::setprecision 和 std::fixed
#include <bitset>
#include "AppLogger.h"
#include "yolodecet/yolo_tracker.h"
using namespace std;

SerialPortHandler::SerialPortHandler(
	boost::asio::io_context& io,
	const std::string& port,
	unsigned int baud_rate
) : serial_(io), connection_type_(ConnectionType::Serial) {
	// 打开串口并配置
	serial_.open(port);
	if (!serial_.is_open()) {
		AppLogger::get().error("Failed to open serial port: {}", port);
	}

	// 设置波特率
	serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

	// 设置字符大小
	serial_.set_option(boost::asio::serial_port_base::character_size(8));

	// 设置停止位
	serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

	// 设置奇偶校验
	serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

	// 设置流量控制
	serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

	AppLogger::get().info("Serial port opened and configured.");

	// 启动发送消息的后台线程
	senderThread = std::thread(&SerialPortHandler::sendMessageLoop, this);
}

// 新增 TCP 构造函数
SerialPortHandler::SerialPortHandler(
	boost::asio::io_context& io,
	const std::string& host,
	unsigned short  port
) : serial_(io), connection_type_(ConnectionType::TCP) {
	// 创建 TCP 套接字并连接
	tcp_socket_ = std::make_shared<boost::asio::ip::tcp::socket>(io);
	boost::asio::ip::tcp::endpoint endpoint(
		boost::asio::ip::make_address(host),
		port
	);
	tcp_socket_->connect(endpoint);
	AppLogger::get().info("Connected to TCP server {}:{}", host, port);

	// 启动发送线程
	senderThread = std::thread(&SerialPortHandler::sendMessageLoop, this);
}

// UDP 构造函数实现
SerialPortHandler::SerialPortHandler(
	boost::asio::io_context& io,
	const std::string& host,
	unsigned short port,
	bool is_udp
) : serial_(io), connection_type_(ConnectionType::UDP) {
	(void)is_udp;  // 避免未使用参数警告

	// 创建 UDP socket
	udp_socket_ = std::make_shared<boost::asio::ip::udp::socket>(io, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));

	// 设置目标端点
	boost::asio::ip::udp::endpoint endpoint(
		boost::asio::ip::make_address(host),
		port
	);

	AppLogger::get().info("UDP socket created, target: {}:{}", host, port);

	// 启动发送线程
	senderThread = std::thread(&SerialPortHandler::sendMessageLoop, this);
}

// 析构函数：确保在对象销毁时正确地停止并等待发送线程结束，防止资源泄漏或程序崩溃。
SerialPortHandler::~SerialPortHandler() {
	running.store(false);
	messageQueue.close(); // 关键：唤醒阻塞的 dequeue

	if (senderThread.joinable()) {
		senderThread.join();
	}
}

/**
 * @brief 将一个MAVLink消息加入到发送队列中。
 *
 * 该方法用于将需要发送的消息添加到线程安全的消息队列中，
 * 后台发送线程会从这个队列中取出消息进行实际的发送操作。
 *
 * @param msg 要发送的MAVLink消息。
 */
void SerialPortHandler::enqueueMessage(mavlink_message_t&& msg) {
	// 使用move语义高效地将msg移动到messageQueue中
	messageQueue.enqueue(std::move(msg));

}


/**
 * @brief 发送消息的后台循环。
 *
 * 这是一个无限循环，检查消息队列中是否有待发送的消息。
 * 如果有，则取出消息并通过send_mavlink_message方法发送出去；
 * 如果没有，则让出当前线程的执行权给其他线程，避免忙等消耗CPU资源。
 */
void SerialPortHandler::sendMessageLoop() {
	// 只要running标志为true，就持续运行循环。
	while (running.load()) {
		// 检查消息队列是否为空，如果不为空，则取出第一个消息进行发送。
		try {
			mavlink_message_t msg = messageQueue.dequeue(); // 从队列中取出消息
			send_mavlink_message(msg); // 发送取出的消息
		}
		catch (const std::exception& e) {
			AppLogger::get().error("Error: {}", e.what());
		}


		//{
		//	// 如果队列为空，调用yield让出控制权给其他线程，避免浪费CPU资源。
		//	std::this_thread::yield();
		//}
	}
}




void SerialPortHandler::start_receive() {
	if (connection_type_ == ConnectionType::Serial) {
		// 串口异步接收
		serial_.async_read_some(
			boost::asio::buffer(read_buffer_),
			[this](const boost::system::error_code& error, size_t bytes_transferred) {
				this->handle_read(error, bytes_transferred);
			}
		);
	}
	else if (connection_type_ == ConnectionType::TCP) {
		// TCP 异步接收
		tcp_socket_->async_read_some(
			boost::asio::buffer(read_buffer_),
			[this](const boost::system::error_code& error, size_t bytes_transferred) {
				this->handle_read(error, bytes_transferred);
			}
		);
	}
	else if (connection_type_ == ConnectionType::UDP) {
		// UDP 异步接收
		udp_socket_->async_receive_from(
			boost::asio::buffer(read_buffer_),
			remote_endpoint_,
			[this](const boost::system::error_code& error, size_t bytes_transferred) {
				this->handle_read(error, bytes_transferred);
			}
		);
	}
}

void SerialPortHandler::send_mavlink_message(const mavlink_message_t& msg) {


	// 打包消息
	uint8_t* buffer = new uint8_t[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	//std::cout << "  - 打包长度: " << len << " 字节" << std::endl;

	// 创建共享数据块，确保异步操作期间 buffer 不被释放
	std::shared_ptr<std::vector<uint8_t>> data =
		std::make_shared<std::vector<uint8_t>>(buffer, buffer + len);

	// === 开始异步发送 ===
	try {
		if (connection_type_ == ConnectionType::Serial) {
			//std::cout << "  - 连接类型: Serial" << std::endl;
			boost::asio::async_write(serial_, boost::asio::buffer(*data),
				[data](const boost::system::error_code& ec, size_t bytes_transferred) {
					if (!ec) {
						//std::cout << "【DEBUG: SERIAL_WRITE】串口发送成功，发送 "
							//<< bytes_transferred << " 字节" << std::endl;
					}
					else {
						AppLogger::get().error("【ERROR: SERIAL_WRITE】串口发送失败: {} (code: {})", ec.message(), ec.value());
					}
				});
		}
		else if (connection_type_ == ConnectionType::TCP) {
			if (tcp_socket_ && tcp_socket_->is_open()) {
				//std::cout << "  - 连接类型: TCP" << std::endl;
				boost::asio::async_write(*tcp_socket_, boost::asio::buffer(*data),
					[data](const boost::system::error_code& ec, size_t bytes_transferred) {
						if (!ec) {
							//std::cout << "【DEBUG: TCP_WRITE】TCP 发送成功，发送 "
						//		<< bytes_transferred << " 字节" << std::endl;
						}
						else {
							AppLogger::get().error("【ERROR: TCP_WRITE】TCP 发送失败: {} (code: {})", ec.message(), ec.value());
						}
					});
			}
			else {
				AppLogger::get().error("【ERROR: TCP_WRITE】TCP socket 未打开或为空！");
			}
		}
		else if (connection_type_ == ConnectionType::UDP) {
			if (udp_socket_ && udp_socket_->is_open()) {
				//std::cout << "  - 连接类型: UDP，目标: "
					//<< remote_endpoint_.address().to_string()
				//	<< ":" << remote_endpoint_.port() << std::endl;

				udp_socket_->async_send_to(boost::asio::buffer(*data), remote_endpoint_,
					[data](const boost::system::error_code& ec, size_t bytes_transferred) {
						if (!ec) {
							//std::cout << "【DEBUG: UDP_SEND】UDP 发送成功，发送 "
							//	<< bytes_transferred << " 字节" << std::endl;
						}
						else {
							AppLogger::get().error("【ERROR: UDP_SEND】UDP 发送失败: {} (code: {})", ec.message(), ec.value());
						}
					});
			}
			else {
				AppLogger::get().error("【ERROR: UDP_SEND】UDP socket 未打开或为空！");
			}
		}
		else {
			AppLogger::get().error("【ERROR: SEND】未知连接类型！");
		}
	}
	catch (const std::exception& e) {
		AppLogger::get().error("【ERROR: SEND】发送过程中发生异常: {}", e.what());
	}
	catch (...) {
		AppLogger::get().error("【ERROR: SEND】发送过程中发生未知异常！");
	}

	//std::cout << "【DEBUG: SEND_EXIT】send_mavlink_message 调用完成（异步发送已启动）" << std::endl;
}

void SerialPortHandler::handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
	if (!error) {
		parse_mavlink_message(MAVLINK_COMM_0, read_buffer_.data(), bytes_transferred);//解析MAVLINK消息
		start_receive(); // 再次启动异步读取
		//// 🔍 打印每次接收到的字节数（用于分析延时/吞吐）
		//std::cout << "[DEBUG] Received " << bytes_transferred << " bytes from "
		//	<< (connection_type_ == ConnectionType::Serial ? "Serial"
		//		: connection_type_ == ConnectionType::TCP ? "TCP"
		//		: "UDP")
		//	<< std::endl;
	}
	else {
		AppLogger::get().error("Read error: {}", error.message());
	}
}

const char* get_result_description(uint8_t result) {
	switch (result) {
	case MAV_RESULT_ACCEPTED:          return "成功 (命令已接受)";
	case MAV_RESULT_TEMPORARILY_REJECTED: return "暂时拒绝 (稍后重试)";
	case MAV_RESULT_DENIED:            return "拒绝 (无权限或参数错误)";
	case MAV_RESULT_UNSUPPORTED:       return "不支持 (系统不支持此命令)";
	case MAV_RESULT_FAILED:            return "失败 (执行过程中发生错误)";
	case MAV_RESULT_IN_PROGRESS:       return "正在进行 (命令正在执行)";
	case MAV_RESULT_CANCELLED:         return "已取消 (命令被用户取消)";
	case MAV_RESULT_COMMAND_LONG_ONLY: return "仅支持 COMMAND_LONG 格式";
	case MAV_RESULT_COMMAND_INT_ONLY:  return "仅支持 COMMAND_INT 格式";
	case MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME: return "不支持的坐标系类型";
	default:                           return "未知结果";
	}
}
const char* get_command_name(uint16_t command_id) {
	switch (command_id) {
	case MAV_CMD_NAV_TAKEOFF:      return "起飞命令 (TAKEOFF)";
	case MAV_CMD_NAV_LAND:         return "着陆命令 (LAND)";
	case MAV_CMD_NAV_RETURN_TO_LAUNCH: return "返航命令 (RTL)";
	case MAV_CMD_COMPONENT_ARM_DISARM: return "解锁/加锁 (ARM/DISARM)";
	case MAV_CMD_DO_SET_MODE:      return "设置模式 (SET_MODE)";
	case MAV_CMD_DO_SET_SERVO:  return "设置伺服器 (SET_SERVO)";
	case MAV_CMD_SET_MESSAGE_INTERVAL: return "设置消息频率 (SET_MESSAGE_INTERVAL)";
	case MAV_CMD_GET_MESSAGE_INTERVAL: return "获取消息频率 (GET_MESSAGE_INTERVAL)";
	case MAV_CMD_DO_CHANGE_SPEED: return "设置速度 (DO_CHANGE_SPEED)";
		// 添加更多常见命令...
	default:                       return "未知命令";
	}
}

void SerialPortHandler::parse_mavlink_message(uint8_t chan, const uint8_t* buffer, size_t len) {
	static mavlink_message_t msg;
	static mavlink_status_t status;

	for (size_t i = 0; i < len; ++i) {
		if (mavlink_parse_char(chan, buffer[i], &msg, &status)) {
			switch (msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT: {
				mavlink_heartbeat_t heartbeat;
				mavlink_msg_heartbeat_decode(&msg, &heartbeat);

				// 解析类型
				const char* type_name = "Unknown";
				switch (heartbeat.type) {
				case MAV_TYPE_FIXED_WING: type_name = "Fixed Wing"; break;
				case MAV_TYPE_QUADROTOR:  type_name = "Quadrotor"; break;
				case MAV_TYPE_GROUND_ROVER: type_name = "Rover"; break;
					// 添加其他类型
				}

				// 解析自动驾驶仪
				const char* autopilot_name = "Unknown";
				switch (heartbeat.autopilot) {
				case MAV_AUTOPILOT_ARDUPILOTMEGA: autopilot_name = "ArduPilot"; break;
				case MAV_AUTOPILOT_PX4:           autopilot_name = "PX4"; break;
				}

				// 解析基本模式
				std::string base_mode_flags;
				if (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)        base_mode_flags += "ARMED ";
				if (heartbeat.base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)base_mode_flags += "MANUAL ";
				if (heartbeat.base_mode & MAV_MODE_FLAG_GUIDED_ENABLED)      base_mode_flags += "GUIDED ";
				if (heartbeat.base_mode & MAV_MODE_FLAG_AUTO_ENABLED)        base_mode_flags += "AUTO ";
				if (heartbeat.base_mode & MAV_MODE_FLAG_TEST_ENABLED)        base_mode_flags += "TEST ";
				if (base_mode_flags.empty()) base_mode_flags = "None";

				// 解析自定义模式（ArduPilot）
				const char* custom_mode_name = "Unknown";
				if (heartbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
					switch (heartbeat.custom_mode) {
					case 0:  custom_mode_name = "STABILIZE"; break;
					case 2:  custom_mode_name = "ALT_HOLD"; break;
					case 3:  custom_mode_name = "AUTO"; break;
					case 4:  custom_mode_name = "GUIDED"; break;
					case 5:  custom_mode_name = "LOITER"; break;
					}
				}

				//// 输出结果
				//std::cout << "心跳: 类型=" << type_name
				//	<< " 自动驾驶=" << autopilot_name
				//	<< " 基本模式=" << base_mode_flags
				//	<< " 自定义模式=" << custom_mode_name
				//	<< std::endl;

				// 将飞行模式数据存入队列
				{
					FlightModeData modeData;
					modeData.base_mode = heartbeat.base_mode;
					modeData.custom_mode = heartbeat.custom_mode;
					std::lock_guard<std::mutex> lock(flight_mode_mutex_);
					latest_flight_mode_ = modeData;
				}
				//std::cout << "  当前接收缓冲区大小: " << sizeof(read_buffer_) << " 字节" << std::endl;
				break;
			}
			case MAVLINK_MSG_ID_COMMAND_ACK: {
				mavlink_command_ack_t command_ack;
				mavlink_msg_command_ack_decode(&msg, &command_ack);

				AppLogger::get().info("接收到 COMMAND_ACK 消息:");

				// 1. 解析命令ID（转换为可读名称）
				AppLogger::get().info("  命令: {} (ID={})", get_command_name(command_ack.command), (int)command_ack.command);

				// 2. 解析结果码（完整覆盖 MAV_RESULT 枚举）
				AppLogger::get().info("  结果: {}", get_result_description(command_ack.result));

				// 3. 处理进度信息（如果结果为 IN_PROGRESS）
				if (command_ack.result == MAV_RESULT_IN_PROGRESS) {
					AppLogger::get().info("  进度: {}%", (int)command_ack.progress);
				}

				break;
			}
			case MAVLINK_MSG_ID_MISSION_REQUEST_INT: {//接受
				mavlink_mission_request_int_t request;
				mavlink_msg_mission_request_int_decode(&msg, &request);

				std::lock_guard<std::mutex> lock(mission_request_int_data_mutex_);
				latest_mission_request_int_ = request;
				break;
			}

			case MAVLINK_MSG_ID_MISSION_REQUEST: {
				mavlink_mission_request_t request;
				mavlink_msg_mission_request_decode(&msg, &request);

				std::lock_guard<std::mutex> lock(mission_request_data_mutex_);
				latest_mission_request_ = request;
				break;
			}
			case MAVLINK_MSG_ID_MISSION_ACK: {
				mavlink_mission_ack_t mission_ack;
				mavlink_msg_mission_ack_decode(&msg, &mission_ack);
				{
					std::lock_guard<std::mutex> lock(mission_ack_data_mutex_);
					latest_mission_ack_ = mission_ack;//添加到队列中
				}

				AppLogger::get().info("任务确认:");
				AppLogger::get().info("  目标系统: {}", (int)mission_ack.target_system);
				AppLogger::get().info("  目标组件: {}", (int)mission_ack.target_component);
				if (mission_ack.type == MAV_MISSION_ACCEPTED) {
					AppLogger::get().info("  结果: 成功");
				}
				else {
					AppLogger::get().info("  结果: 失败, 错误码: {}", (int)mission_ack.type);
				}
				AppLogger::get().info("  任务类型: {}", (int)mission_ack.mission_type);
				if (mission_ack.opaque_id != 0) {
					AppLogger::get().info("  任务ID: {}", mission_ack.opaque_id);
				}
				else {
					AppLogger::get().info("  任务ID: 0");
				}
				break;
			}
			case MAVLINK_MSG_ID_MESSAGE_INTERVAL: {
				mavlink_message_interval_t interval_msg;
				mavlink_msg_message_interval_decode(&msg, &interval_msg);

				uint16_t msg_id = interval_msg.message_id;
				int32_t interval_us = interval_msg.interval_us; // 注意：是 int32_t，-1 表示禁用

				AppLogger::get().info("收到 MESSAGE_INTERVAL 响应:");
				AppLogger::get().info("  消息 ID: {}", msg_id);

				if (interval_us == -1) {
					AppLogger::get().info("  状态: 已禁用");
				}
				else if (interval_us == 0) {
					AppLogger::get().info("  状态: 使用默认频率");
				}
				else {
					float freq = 1e6f / interval_us;
					AppLogger::get().info("  间隔: {} 微秒 ({:.1f} Hz)", interval_us, freq);
				}

				break;
			}
			case MAVLINK_MSG_ID_MISSION_CURRENT: {
				mavlink_mission_current_t current;
				mavlink_msg_mission_current_decode(&msg, &current);

				std::lock_guard<std::mutex> lock(mission_current_mutex_);
				latest_mission_current_ = current;
				has_new_mission_current_ = true;  // ✅ 标记为有新数据

				// 可选：打印日志
				// std::cout << "更新当前航点: " << current.seq << std::endl;
				break;
			}
			case MAVLINK_MSG_ID_ATTITUDE: {
				mavlink_attitude_t attitude;
				mavlink_msg_attitude_decode(&msg, &attitude);

				{
					std::lock_guard<std::mutex> lock(imu_data_mutex_);
					latest_imu_data_ = attitude;
				}
				data_cv_.notify_one(); // 可选：如果主线程在等待新数据，则通知它

				/*std::cout << "ATTITUDE 消息:" <<std::endl;
				std::cout << "系统启动时间 (ms): " << attitude.time_boot_ms;
				std::cout << "横滚角: " << (float)attitude.roll * 180.00000 / 3.1415926;
				std::cout << "俯仰角: " << (float)attitude.pitch * 180.00000 / 3.1415926;
				std::cout << "偏航角: " << (float)attitude.yaw * 180.00000 / 3.1415926;
				std::cout << "横滚角速度: " << (float)attitude.rollspeed * 180.00000 / 3.1415926;
				std::cout << "俯仰角速度: " << (float)attitude.pitchspeed * 180.00000 / 3.1415926;
				std::cout << "偏航角速度: " << (float)attitude.yawspeed * 180.00000 / 3.1415926 <<std::endl;*/
				break;
			}
			case MAVLINK_MSG_ID_GPS_RAW_INT: {//接收gps
				mavlink_gps_raw_int_t gps_raw;
				mavlink_msg_gps_raw_int_decode(&msg, &gps_raw);
				{
					std::lock_guard<std::mutex> lock(gps_raw_data_mutex_);
					latest_gps_raw_data_ = gps_raw;
				}
				data_cv_.notify_one();

				//std::cout << std::fixed << std::setprecision(7);
				//std::cout << "  纬度 (degE7): " << (double)gps_raw.lat / 1e7 << std::endl;
				//std::cout << "  经度 (degE7): " << (double)gps_raw.lon / 1e7 << std::endl;
				//if (gps_raw.h_acc != UINT32_MAX) {
				//	float h_accuracy = gps_raw.h_acc / 1000.0f; // mm → meters
				//	std::cout << "水平精度 (h_acc): " << h_accuracy << " 米" << std::endl;
				//}

				//if (gps_raw.v_acc != UINT32_MAX) {
				//	float v_accuracy = gps_raw.v_acc / 1000.0f;
				//	std::cout << "垂直精度: " << v_accuracy << " 米" << std::endl;
				//}

				//// 4. 卫星数量
				//std::cout << "卫星数量: " << (int)gps_raw.satellites_visible << std::endl;

				break;
			}
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
				// 创建一个 GLOBAL_POSITION_INT 消息结构体
				mavlink_global_position_int_t global_pos;
				
				// 解码接收到的消息到结构体中
				mavlink_msg_global_position_int_decode(&msg, &global_pos);               
				//distance_to_ground.store(global_pos.relative_alt/10-47.6);///获取距离//模拟数据
				{
					std::lock_guard<std::mutex> lock(gps_data_mutex_);
					latest_gps_data_ = global_pos;
				}
				data_cv_.notify_one(); // 可选：如果主线程在等待新数据，则通知它

				// // 打印解码后的信息
				//std::cout << "全局位置 (整数): " <<std::endl;
				//std::cout << "  系统启动时间 (ms): " << (uint32_t)global_pos.time_boot_ms <<std::endl;
				// // 设置精度为7位小数
				// std::cout << std::fixed << std::setprecision(7);
				//std::cout << "  纬度 (degE7): " << (double)global_pos.lat / 1e7 <<std::endl;
				//std::cout << "  经度 (degE7): " << (double)global_pos.lon / 1e7 <<std::endl;
				//std::cout << "  海拔 (mm): " << (int32_t)global_pos.alt <<std::endl;
				//std::cout << "  相对海拔 (mm): " << (int32_t)global_pos.relative_alt <<std::endl;
				//std::cout << "  地面X速度 (cm/s): " << (int16_t)global_pos.vx <<std::endl;
				//std::cout << "  地面Y速度 (cm/s): " << (int16_t)global_pos.vy <<std::endl;
				//std::cout << "  地面Z速度 (cm/s): " << (int16_t)global_pos.vz <<std::endl;
				//std::cout << "  航向 (cdeg): " << (uint16_t)global_pos.hdg  / 100.0f<<std::endl;
				// // 如果航向未知
				// if (global_pos.hdg == UINT16_MAX) {
				//    std::cout << "  航向: 未知" <<std::endl;
				// }

				break;
			}
			case MAVLINK_MSG_ID_RC_CHANNELS: {
				mavlink_rc_channels_t rc_channels;
				mavlink_msg_rc_channels_decode(&msg, &rc_channels);

				{
					std::lock_guard<std::mutex> lock(rc_channels_mutex_);
					latest_rc_channels_ = rc_channels;
				}
				data_cv_.notify_one();//传递遥控通道值

				/*std::cout << "遥控器通道: "
					<< "时间=" << rc_channels.time_boot_ms << "ms "
					<< "通道数=" << (int)rc_channels.chancount << " "
					<< "通道值=["
					<< rc_channels.chan1_raw << ", "
					<< rc_channels.chan2_raw << ", "
					<< rc_channels.chan3_raw << ", "
					<< rc_channels.chan4_raw << ", "
					<< rc_channels.chan5_raw << ", "
					<< rc_channels.chan6_raw << ", "
					<< rc_channels.chan7_raw << ", "
					<< rc_channels.chan8_raw << "] "
					<< "RSSI=" << (int)rc_channels.rssi << "%"
					<< std::endl;*/
				break;
			}
			case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
				mavlink_servo_output_raw_t servo_output = {};
				mavlink_msg_servo_output_raw_decode(&msg, &servo_output);
				{
					std::lock_guard<std::mutex> lock(servo_output_mutex_);
					latest_servo_output_ = servo_output;
				}
				//// 调试输出部分（可选地放在线程安全区域外）
				//std::cout << "[DEBUG] Servo Output Raw:" << std::endl;
				//std::cout << "  TimeUS: " << servo_output.time_usec << std::endl;
				//std::cout << "  Servo1: " << servo_output.servo1_raw << std::endl;
				//std::cout << "  Servo2: " << servo_output.servo2_raw << std::endl;
				//std::cout << "  Servo3: " << servo_output.servo3_raw << std::endl;
				//std::cout << "  Servo4: " << servo_output.servo4_raw << std::endl;
				//std::cout << "  Servo5: " << servo_output.servo5_raw << std::endl;
				//std::cout << "  Servo6: " << servo_output.servo6_raw << std::endl;
				//std::cout << "  Servo7: " << servo_output.servo7_raw << std::endl;
				//std::cout << "  Servo8: " << servo_output.servo8_raw << std::endl;

				break;
			}
			case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: {
				mavlink_position_target_local_ned_t target_ned;
				mavlink_msg_position_target_local_ned_decode(&msg, &target_ned);

				// 解析类型掩码（确定哪些字段有效）
				uint16_t type_mask = target_ned.type_mask;
				std::bitset<16> mask_bits(type_mask);
				// 检查位置是否有效
				if (!(type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE)) {
					AppLogger::get().debug("  目标位置: 北={}m, 东={}m, 地={}m", target_ned.x, target_ned.y, target_ned.z);
				}

				// 检查速度是否有效
				if (!(type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE)) {
					AppLogger::get().debug("  目标速度: 北={}m/s, 东={}m/s, 地={}m/s", target_ned.vx, target_ned.vy, target_ned.vz);
				}

				// 检查偏航角是否有效
				if (!(type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)) {
					AppLogger::get().debug("  目标偏航角: {}rad", target_ned.yaw);
				}
				if (type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) {
					AppLogger::get().debug("  偏航角速率: {}rad/s", target_ned.yaw_rate);
				}
				
				break;
			}
			case MAVLINK_MSG_ID_DISTANCE_SENSOR: {// 距离传感器
				mavlink_distance_sensor_t distance_sensor;
				mavlink_msg_distance_sensor_decode(&msg, &distance_sensor);
				if (distance_sensor.type == MAV_DISTANCE_SENSOR_LASER) {
					AppLogger::get().debug("  距离 (cm): {}", distance_sensor.current_distance);
					distance_to_ground.store(distance_sensor.current_distance );///获取距离
					{
						std::lock_guard<std::mutex> lock(distance_sensor_data_mutex_);
						latest_distance_sensor_ = distance_sensor;
					}
				}
				break;
			}
			case MAVLINK_MSG_ID_HOME_POSITION: {
				mavlink_home_position_t home_pos;
				mavlink_msg_home_position_decode(&msg, &home_pos);

				HomePositionData home_data;
				home_data.latitude = home_pos.latitude / 1e7;
				home_data.longitude = home_pos.longitude / 1e7;
				home_data.altitude = home_pos.altitude / 1000.0f; // mm转m

				{
					std::lock_guard<std::mutex> lock(home_position_mutex_);
					latest_home_position_ = home_data;
				}
				/*std::cout << "获取到起飞点坐标: "
					<< std::setprecision(7)
					<< "纬度=" << home_data.latitude
					<< " 经度=" << home_data.longitude
					<< " 海拔=" << home_data.altitude << "m" << std::endl;*/
				break;
			}
			default:
				//cout << "未知消息类型" <<std::endl;
				break;
			}
		}
	}
}

//对地传感器数据进行获取
bool SerialPortHandler::try_get_distance_sensor_data(mavlink_distance_sensor_t& distance_sensor_data) {
	std::lock_guard<std::mutex> lock(distance_sensor_data_mutex_);
	if (latest_distance_sensor_.has_value()) {
		distance_sensor_data = *latest_distance_sensor_;

		return true;
	}
	return false;
}

bool SerialPortHandler::try_get_mission_ack_data(mavlink_mission_ack_t& mission_ack_data) {
	std::lock_guard<std::mutex> lock(mission_ack_data_mutex_);
	if (latest_mission_ack_.has_value()) {
		mission_ack_data = *latest_mission_ack_;

		return true;
	}
	return false;
}
bool SerialPortHandler::try_get_mission_request_int_data(mavlink_mission_request_int_t& request_data) {
	std::lock_guard<std::mutex> lock(mission_request_int_data_mutex_);
	if (latest_mission_request_int_.has_value()) {
		request_data = *latest_mission_request_int_;

		return true;
	}
	return false;
}
bool SerialPortHandler::try_get_mission_request_data(mavlink_mission_request_t& request_data) {
	std::lock_guard<std::mutex> lock(mission_request_data_mutex_);
	if (latest_mission_request_.has_value()) {
		request_data = *latest_mission_request_;

		return true;
	}
	return false;
}

bool SerialPortHandler::try_get_gps_data(mavlink_global_position_int_t& gps_data) {
	std::lock_guard<std::mutex> lock(gps_data_mutex_);
	if (latest_gps_data_.has_value()) {
		gps_data = *latest_gps_data_;

		return true;
	}
	return false;
}
bool SerialPortHandler::try_get_gps_raw_data(mavlink_gps_raw_int_t& gps_raw_data) {
	std::lock_guard<std::mutex> lock(gps_raw_data_mutex_);
	if (latest_gps_raw_data_.has_value()) {
		gps_raw_data = *latest_gps_raw_data_;

		return true;
	}
	return false;
}//获取GPS数据精确

bool SerialPortHandler::try_get_rc_channels(mavlink_rc_channels_t& rc_channels) {
	std::lock_guard<std::mutex> lock(rc_channels_mutex_);
	if (latest_rc_channels_.has_value()) {
		rc_channels = *latest_rc_channels_;

		return true;
	}
	return false;
}

bool SerialPortHandler::try_get_imu_data(mavlink_attitude_t& imu_data) {
	std::lock_guard<std::mutex> lock(imu_data_mutex_);
	if (latest_imu_data_.has_value()) {
		imu_data = *latest_imu_data_;

		return true;
	}
	return false;
}
bool SerialPortHandler::try_get_servo_pwm(uint8_t servo_index, uint16_t& pwm) {
	std::lock_guard<std::mutex> lock(servo_output_mutex_);
	if (!latest_servo_output_.has_value()) {
		return false;
	}

	const auto& data = *latest_servo_output_;
	switch (servo_index) {
	case 1: pwm = data.servo1_raw; break;
	case 2: pwm = data.servo2_raw; break;
	case 3: pwm = data.servo3_raw; break;
	case 4: pwm = data.servo4_raw; break;
	case 5: pwm = data.servo5_raw; break;
	case 6: pwm = data.servo6_raw; break;
	case 7: pwm = data.servo7_raw; break;
	case 8: pwm = data.servo8_raw; break;
	case 9: pwm = data.servo9_raw; break;
	case 10: pwm = data.servo10_raw; break;
	case 11: pwm = data.servo11_raw; break;
	case 12: pwm = data.servo12_raw; break;
	case 13: pwm = data.servo13_raw; break;
	case 14: pwm = data.servo14_raw; break;
	case 15: pwm = data.servo15_raw; break;
	case 16: pwm = data.servo16_raw; break;

	default:
		return false;
	}

	return true;
}

// 实现try_get_flight_mode方法
bool SerialPortHandler::try_get_flight_mode(FlightModeData& mode) {
	std::lock_guard<std::mutex> lock(flight_mode_mutex_);
	if (latest_flight_mode_.has_value()) {
		mode = *latest_flight_mode_;

		return true;
	}
	return false;
}

// 实现获取home_position
bool SerialPortHandler::try_get_home_position(HomePositionData& home_pos) {
	std::lock_guard<std::mutex> lock(home_position_mutex_);
	if (latest_home_position_.has_value()) {
		home_pos = *latest_home_position_;

		return true;
	}
	return false;
}

void SerialPortHandler::setup_pwm_message(mavlink_message_t& msg, uint16_t pwm_value, uint16_t servo_n) {
	//uint16_t servo_number = 6; //控制的伺服编号
	//uint16_t pwm_value = 1900; // PWM值，范围通常是1000到2000

	// 使用command_long消息类型发送设置伺服指令
	mavlink_msg_command_long_pack(system_id, component_id, &msg,
		target_system, target_component,
		MAV_CMD_DO_SET_SERVO, // 设置伺服指令
		0, // 确认与否，0表示不需要确认
		servo_n, // 第几个servo
		pwm_value, // pwm值
		0, 0, 0, 0, 0); // 其余参数不使用
}

void SerialPortHandler::setup_flight_mode_message(mavlink_message_t& msg, uint8_t flight_mode) {
	// 使用 COMMAND_LONG 消息类型发送更改飞行模式指令
	mavlink_msg_command_long_pack(
		system_id,          // 本机系统 ID
		component_id,       // 本机组件 ID
		&msg,               // 输出的 MAVLink 消息
		target_system,      // 目标系统 ID
		target_component,   // 目标组件 ID
		MAV_CMD_DO_SET_MODE, // 设置飞行模式指令
		0,                  // 确认与否，0 表示不需要确认
		MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // 主模式标志位（启用自定义模式）
		flight_mode,        // 目标飞行模式值ArduPilot 飞行模式
		/*以下是常见的 ArduPilot 飞行模式值：

		模式名称	值
		STABILIZE	0
		ACRO	1
		ALT_HOLD	2
		AUTO	3
		GUIDED	4
		LOITER	5
		RTL	6
		CIRCLE	7
		LAND	9
		20*/
		0, 0, 0, 0, 0       // 其余参数不使用
	);
}
/**
 * @brief 设置飞行器飞行模式（通过 COMMAND_LONG 发送 MAV_CMD_DO_SET_MODE）
 *
 * 该函数封装了 MAVLink 消息的创建、打包和入队过程。
 * 用户只需传入目标飞行模式值（例如 4 表示 GUIDED），无需手动管理消息结构。
 *
 * @param flight_mode 目标飞行模式（ArduPilot 定义的值：0=STABILIZE, 4=GUIDED, 6=RTL 等）
 * @return true 成功打包并入队；false 打包失败或入队失败
 */
bool SerialPortHandler::setFlightMode(uint8_t flight_mode) {
	// 1. 创建要发送的 COMMAND_LONG 消息结构
	mavlink_message_t cmd{};
	bool success =mavlink_msg_command_long_pack(
		system_id,          // 本机系统 ID
		component_id,       // 本机组件 ID
		&cmd,               // 输出的 MAVLink 消息
		target_system,      // 目标系统 ID
		target_component,   // 目标组件 ID
		MAV_CMD_DO_SET_MODE, // 设置飞行模式指令
		0,                  // 确认与否，0 表示不需要确认
		MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, // 主模式标志位（启用自定义模式）
		flight_mode,        // 目标飞行模式值ArduPilot 飞行模式
		/*以下是常见的 ArduPilot 飞行模式值：

		模式名称	值
		STABILIZE	0
		ACRO	1
		ALT_HOLD	2
		AUTO	3
		GUIDED	4
		LOITER	5
		RTL	6
		CIRCLE	7
		LAND	9
		20*/
		0, 0, 0, 0, 0       // 其余参数不使用
	);

	if (!success) {
		//cout << "Failed to pack COMMAND_LONG message." << endl;
		return false; // 打包失败
	}

	// 3. 尝试将消息加入发送队列
	
	if(	messageQueue.enqueue(std::move(cmd))) {// 使用移动语义高效入队
		//cout << "COMMAND_LONG message enqueued.成功入队" << endl;
		return true; // 成功入队
	}
	else
	{
		AppLogger::get().error("Failed to enqueue COMMAND_LONG message.");
		return false; // 入队失败（例如队列满或异常）
	}
}

void SerialPortHandler::setup_guided_velocity(mavlink_message_t& msg, float vx, float vy, float vz, float toyaw_rate) {
	uint32_t time_boot_ms = 0; // 时间戳（如果不使用，可以设置为 0）
	// 修改类型掩码确保所有不需要的字段被正确忽略
	uint16_t type_mask = (POSITION_TARGET_TYPEMASK_X_IGNORE |
		POSITION_TARGET_TYPEMASK_Y_IGNORE |
		POSITION_TARGET_TYPEMASK_Z_IGNORE |
		POSITION_TARGET_TYPEMASK_AX_IGNORE |
		POSITION_TARGET_TYPEMASK_AY_IGNORE |
		POSITION_TARGET_TYPEMASK_AZ_IGNORE |
		POSITION_TARGET_TYPEMASK_YAW_IGNORE);

	// 设置目标速度
	mavlink_msg_set_position_target_local_ned_pack(
		system_id,          // 本机系统 ID
		component_id,       // 本机组件 ID
		&msg,               // 输出的 MAVLink 消息
		time_boot_ms,       // 时间戳
		target_system,      // 目标系统 ID
		target_component,   // 目标组件 ID
		MAV_FRAME_BODY_NED,   // 坐标系（局部坐标系）
		type_mask,          // 类型掩码
		0, 0, 0,            // 目标位置（不使用）
		vx, vy, vz,         // 目标速度（X, Y, Z）
		0, 0, 0,            // 目标加速度（不使用）
		0, toyaw_rate         // 目标偏航角和偏航角速度
	);
}

/**
 * @brief 构造设置目标位置的 MAVLink 消息（局部坐标系）
 * @param msg 输出的 MAVLink 消息
 * @param target_system 目标系统的 ID
 * @param target_component 目标组件的 ID
 * @param x 目标位置的 X 坐标（单位：米）
 * @param y 目标位置的 Y 坐标（单位：米）
 * @param z 目标位置的 Z 坐标（单位：米）
 * @param yaw 目标偏航角（单位：弧度）
 */
void SerialPortHandler::setup_guided_position_local(mavlink_message_t& msg, float x, float y, float z, float yaw) {
	uint32_t time_boot_ms = 0; // 时间戳（如果不使用，可以设置为 0）
	uint16_t type_mask = 0;    // 类型掩码（0 表示使用所有字段）

	// 设置目标位置
	mavlink_msg_set_position_target_local_ned_pack(
		system_id,          // 本机系统 ID
		component_id,       // 本机组件 ID
		&msg,               // 输出的 MAVLink 消息
		time_boot_ms,       // 时间戳
		target_system,      // 目标系统 ID
		target_component,   // 目标组件 ID
		MAV_FRAME_LOCAL_NED, // 坐标系（局部坐标系）
		type_mask,          // 类型掩码
		x, y, z,            // 目标位置（X, Y, Z）
		0, 0, 0,            // 目标速度（不使用）
		0, 0, 0,            // 目标加速度（不使用）
		yaw, 0              // 目标偏航角和偏航角速度
	);
}
void SerialPortHandler::setup_guided_position_gps(mavlink_message_t& msg, double  lat_deg, double  lon_deg, float  alt_amsl, float yaw) {
	uint32_t time_boot_ms = 0; // 可选：使用实际时间戳

	// 设置类型掩码：只使用位置（lat, lon, alt），忽略速度、加速度，保持 Yaw 不变
	uint16_t type_mask = (POSITION_TARGET_TYPEMASK_VX_IGNORE |
		POSITION_TARGET_TYPEMASK_VY_IGNORE |
		POSITION_TARGET_TYPEMASK_VZ_IGNORE |
		POSITION_TARGET_TYPEMASK_AX_IGNORE |
		POSITION_TARGET_TYPEMASK_AY_IGNORE |
		POSITION_TARGET_TYPEMASK_AZ_IGNORE |
		POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE); // 不使用偏航角速度 // 保持当前偏航角POSITION_TARGET_TYPEMASK_YAW_IGNORE |

	float alt_amsl_m = alt_amsl / 1000.0f;
	// ✅ 关键：转换为 degE7（int32_t）
	int32_t lat_e7 = static_cast<int32_t>(lat_deg * 1e7);
	int32_t lon_e7 = static_cast<int32_t>(lon_deg * 1e7);
	yaw=yaw*M_PI/180.0f;
	// 使用 GLOBAL_INT 坐标系（支持高精度 GPS）
	mavlink_msg_set_position_target_global_int_pack(
		system_id,
		component_id,
		&msg,
		time_boot_ms,
		target_system,
		target_component,
		MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,        // 使用 GLOBAL_INT，支持整数型 GPS 坐标
		type_mask,
		lat_e7,                     // 直接传入 int32_t lat (deg * 1e7)
		lon_e7,                // 直接传入 int32_t lon (deg * 1e7)
		alt_amsl,                  // 高度转为米
		0, 0, 0,                     // vx, vy, vz (ignored)
		0, 0, 0,                     // ax, ay, az (ignored)
		yaw, 0                         // yaw, yaw_rate (ignored → 保持当前偏航)
	);
}
// 在 SerialPortHandler.cpp 中实现
void SerialPortHandler::setup_guided_position_body(mavlink_message_t& msg, float x, float y, float z, float yaw) {
	uint32_t time_boot_ms = 0;
	uint16_t type_mask = 0;
	type_mask |= POSITION_TARGET_TYPEMASK_VX_IGNORE;     // 忽略 x 速度
	type_mask |= POSITION_TARGET_TYPEMASK_VY_IGNORE;     // 忽略 y 速度  
	type_mask |= POSITION_TARGET_TYPEMASK_VZ_IGNORE;     // 忽略 z 速度
	type_mask |= POSITION_TARGET_TYPEMASK_AX_IGNORE;     // 忽略 x 加速度
	type_mask |= POSITION_TARGET_TYPEMASK_AY_IGNORE;     // 忽略 y 加速度
	type_mask |= POSITION_TARGET_TYPEMASK_AZ_IGNORE;     // 忽略 z 加速度
	//type_mask |= POSITION_TARGET_TYPEMASK_FORCE_SET;     // 使用加速度而不是力（或者设置为0忽略力）
	//type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;    // 忽略偏航角
	type_mask |= POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE; // 忽略偏航率
	yaw = yaw * M_PI / 180.0f;
	mavlink_msg_set_position_target_local_ned_pack(
		system_id,
		component_id,
		&msg,
		time_boot_ms,
		target_system,
		target_component,
		MAV_FRAME_BODY_NED,  // 使用机体坐标系
		type_mask,
		x, y, z,
		0, 0, 0,
		0, 0, 0,
		0, 0
	);
}

// 静态成员定义
mavlink_mission_current_t SerialPortHandler::latest_mission_current_;
std::atomic<bool> SerialPortHandler::has_new_mission_current_{ false };
std::mutex SerialPortHandler::mission_current_mutex_;

// 获取最新值（只会拿到一次最新数据）
bool SerialPortHandler::try_get_latest_mission_current(mavlink_mission_current_t& current) {
	if (has_new_mission_current_) {
		std::lock_guard<std::mutex> lock(mission_current_mutex_);
		current = latest_mission_current_;
		has_new_mission_current_ = false;  
		return true;
	}
	return false;
}

// 清空（可选）
void SerialPortHandler::clear_mission_current() {
	has_new_mission_current_ = false;
}