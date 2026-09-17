// FrameDispatcher.cpp
#include "FrameDispatcher.h"

FrameDispatcher frameDispatcher;

FrameDispatcher::FrameDispatcher()
    : current_frame_(nullptr) {
}

void FrameDispatcher::updateFrame(const cv::Mat& frame) {
    auto new_frame = std::make_shared<cv::Mat>(frame.clone());

    {
        std::lock_guard<std::mutex> lock(staging_mutex_);
        staging_frame_ = std::move(new_frame);
    }

    frame_counter_.fetch_add(1, std::memory_order_relaxed);
}

void FrameDispatcher::updateFrame(cv::Mat&& frame) {
    auto new_frame = std::make_shared<cv::Mat>(std::move(frame));

    {
        std::lock_guard<std::mutex> lock(staging_mutex_);
        staging_frame_ = std::move(new_frame);
    }

    frame_counter_.fetch_add(1, std::memory_order_relaxed);
}

std::shared_ptr<const cv::Mat> FrameDispatcher::getFrame() const {
    std::shared_ptr<cv::Mat> local_frame;

    // 从 staging 区取出最新帧
    {
        std::lock_guard<std::mutex> lock(staging_mutex_);
        if (staging_frame_) {
            local_frame = std::move(staging_frame_);
        }
    }

    // 如果取到新帧，原子更新 current_frame_
    if (local_frame) {
        //现在可以调用 store() 了，因为 current_frame_ 是 mutable
        current_frame_.store(local_frame, std::memory_order_release);
        // 返回 const 视图
        return std::const_pointer_cast<const cv::Mat>(local_frame);
    }

    // 否则返回当前帧的 const 视图
    auto raw_ptr = current_frame_.load(std::memory_order_acquire);
    if (raw_ptr) {
        return std::const_pointer_cast<const cv::Mat>(raw_ptr);
    }

    return nullptr;
}

uint64_t FrameDispatcher::getFrameCounter() const {
    return frame_counter_.load(std::memory_order_relaxed);
}