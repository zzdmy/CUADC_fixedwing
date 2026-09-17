// realtime_pipeline.h
#pragma once

#include <vector>
#include <memory>
#include <thread>
#include <opencv2/opencv.hpp>

// YOLO 检测结果结构
struct yoloout {
	std::string name;
	float confidence;
	cv::Mat tjImage;
	cv::Point corner;
	cv::Rect box;
	int classId;
};

// 初始化数据管道
void init_pipeline();

// 推送 YOLO 检测结果（线程安全，自动丢弃旧数据）
void push_yolo(const std::vector<yoloout>& outputs);
void push_yolo(std::vector<yoloout>&& outputs); // 移动语义

// 获取最新的 YOLO 检测结果（返回 shared_ptr，无数据时返回 nullptr）
std::shared_ptr<std::vector<yoloout>> get_latest_yolo();

// 关闭管道，通知所有等待中的线程退出
void shutdown_pipeline();
void signal_stop();