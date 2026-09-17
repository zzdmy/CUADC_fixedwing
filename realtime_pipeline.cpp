// realtime_pipeline.cpp
#include "realtime_pipeline.h"
#include <mutex>
#include <condition_variable>
#include <atomic>

template<typename T>
class LatestHolder {
private:
	mutable std::mutex m_mutex;
	std::condition_variable m_cv;
	std::shared_ptr<T> m_data;
	std::atomic<bool> m_has_new_data{ false };
	std::atomic<bool> m_shutdown{ false };  // 用于通知退出等待

public:
	void update(std::shared_ptr<T> new_data) {
		if (!new_data || m_shutdown.load()) return;
		{
			std::scoped_lock lock(m_mutex);
			m_data = std::move(new_data);
			m_has_new_data.store(true, std::memory_order_release);
		}
		m_cv.notify_all();
	}

	// 尝试获取最新数据
	std::shared_ptr<T> try_get_latest() const {
		if (!m_has_new_data.load(std::memory_order_acquire)) return nullptr;
		std::scoped_lock lock(m_mutex);
		return m_data;
	}

	void shutdown() {
		m_shutdown.store(true, std::memory_order_relaxed);
		m_cv.notify_all();
	}

	~LatestHolder() {
		shutdown();
	}
};

// 全局实例
static LatestHolder<std::vector<yoloout>>* g_yoloHolder = nullptr;

// -------------------------------
// 外部接口函数
// -------------------------------

void init_pipeline() {
	g_yoloHolder = new LatestHolder<std::vector<yoloout>>();
}

void push_yolo(const std::vector<yoloout>& outputs) {
	if (!g_yoloHolder) return;
	auto shared_outputs = std::make_shared<std::vector<yoloout>>(outputs);
	g_yoloHolder->update(shared_outputs);
}
void push_yolo(std::vector<yoloout>&& outputs) {
	if (!g_yoloHolder) return;
	auto shared_outputs = std::make_shared<std::vector<yoloout>>(std::move(outputs));
	g_yoloHolder->update(shared_outputs);
}
std::shared_ptr<std::vector<yoloout>> get_latest_yolo() {
	if (!g_yoloHolder) return nullptr;
	return g_yoloHolder->try_get_latest();
}

void shutdown_pipeline() {
	if (g_yoloHolder) {
		g_yoloHolder->shutdown();
		delete g_yoloHolder;
		g_yoloHolder = nullptr;
	}
}

// 供外部调用以通知停止
void signal_stop() {
	shutdown_pipeline();
}