// FrameDispatcher.h
#pragma once

#include <opencv2/opencv.hpp>
#include <atomic>
#include <memory>
#include <mutex>

/**
 * @brief 单例式帧分发器类，用于线程安全地更新和获取最新的 OpenCV 图像帧。
 *
 * 该类允许多个生产者线程通过 updateFrame() 提交新帧，
 * 同时允许多个消费者线程通过 getFrame() 安全地读取最新帧的只读副本。
 * 内部使用原子智能指针实现无锁读取，写入操作通过互斥锁保护临时缓冲区以避免频繁分配。
 *
 * 注意：本类设计为全局单例（通过外部定义的 frameDispatcher 实例使用）。
 */
class FrameDispatcher {
public:
    /**
     * @brief 构造函数。
     *
     * 初始化帧计数器和内部状态。默认无有效帧。
     */
    FrameDispatcher();

    /**
     * @brief 更新当前帧（拷贝语义）。
     *
     * 将输入帧深拷贝到内部缓冲区，并原子地发布为最新帧。
     * 此版本适用于左值（如已存在的 cv::Mat 对象）。
     *
     * @param[in] frame 要更新的图像帧（将被深拷贝）。
     */
    void updateFrame(const cv::Mat& frame);

    /**
     * @brief 更新当前帧（移动语义）。
     *
     * 接收输入帧的所有权（若可能），避免不必要的拷贝。
     * 此版本适用于右值（如临时 cv::Mat 对象）。
     *
     * @param[in] frame 要更新的图像帧（可能被移动）。
     */
    void updateFrame(cv::Mat&& frame);

    /**
     * @brief 获取当前最新帧的只读共享指针。
     *
     * 此操作是线程安全的且无锁（基于 std::atomic_load）。
     * 返回的 shared_ptr 指向 const cv::Mat，防止调用者意外修改帧内容。
     *
     * @return 指向最新帧的 shared_ptr<const cv::Mat>；若尚无帧，则返回 nullptr。
     */
    std::shared_ptr<const cv::Mat> getFrame() const;

    /**
     * @brief 获取自对象创建以来累计处理的帧总数。
     *
     * 每次成功调用 updateFrame() 后计数器递增。
     *
     * @return 当前帧计数器的值。
     */
    uint64_t getFrameCounter() const;

private:
    /**
     * @brief 原子存储当前发布的帧。
     *
     * 使用 mutable 允许在 const 成员函数（如 getFrame()）中执行原子 store/load。
     * 指向的 cv::Mat 由 shared_ptr 管理生命周期，确保读取线程安全。
     */
    mutable std::shared_ptr<cv::Mat> current_frame_{ nullptr };  // 由 frame_mutex_ 保护（GCC11 无 std::atomic<shared_ptr>）

    /**
     * @brief 帧更新计数器，每次 updateFrame 成功后递增。
     */
    std::atomic<uint64_t> frame_counter_{ 0 };

    /**
     * @brief 临时帧缓冲区，用于在加锁期间暂存待发布的帧。
     *
     * 避免在持有锁时进行昂贵的 Mat 拷贝或分配，提升并发性能。
     */
    std::shared_ptr<cv::Mat> staging_frame_ = nullptr;

    /**
     * @brief 保护 staging_frame_ 的互斥锁。
     *
     * mutable 允许在 const 上下文中锁定（虽然此处未在 const 函数中使用，但保持一致性）。
     */
    mutable std::mutex staging_mutex_;

    /**
     * @brief 保护 current_frame_ 的互斥锁。
     */
    mutable std::mutex frame_mutex_;
};

/**
 * @brief 全局唯一的 FrameDispatcher 实例。
 *
 * 应在对应的 .cpp 文件中定义该实例，供整个程序共享使用。
 */
extern FrameDispatcher frameDispatcher;