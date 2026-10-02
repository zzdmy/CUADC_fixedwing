// ocr_test/probe2_mp4.cpp
// 第二轮探测：寻找可用的硬件 H.264 MP4 管线 + 回读验证已写文件
#include <cstdio>
#include <string>
#include <opencv2/opencv.hpp>

static void tryGst(const char* name, const std::string& pipe, const cv::Size& sz) {
    cv::VideoWriter w(pipe, cv::CAP_GSTREAMER, 0, 30, sz);
    std::printf("%-14s opened=%d\n", name, w.isOpened() ? 1 : 0);
    if (w.isOpened()) {
        for (int i = 0; i < 30; ++i) {
            cv::Mat f(sz, CV_8UC3, cv::Scalar(i * 8, 100, 200 - i * 5));
            w.write(f);
        }
        w.release();
    }
}

static void readback(const char* path) {
    cv::VideoCapture c(path);
    double n = c.get(cv::CAP_PROP_FRAME_COUNT);
    cv::Mat f;
    bool ok = c.isOpened() && (c.read(f) || !f.empty());
    std::printf("readback %-22s open=%d frames~%.0f firstFrame=%d\n",
        path, c.isOpened() ? 1 : 0, n, (!f.empty()) ? 1 : 0);
}

int main() {
    const cv::Size sz(1920, 1080);

    tryGst("hw1(NV12)",
        "appsrc ! videoconvert ! video/x-raw,format=NV12 ! nvvidconv ! nvv4l2h264enc ! h264parse ! qtmux ! filesink location=/tmp/p_hw1.mp4", sz);
    tryGst("hw2(I420)",
        "appsrc ! videoconvert ! video/x-raw,format=I420 ! nvvidconv ! nvv4l2h264enc ! h264parse ! qtmux ! filesink location=/tmp/p_hw2.mp4", sz);
    tryGst("hw3(NVMM)",
        "appsrc ! videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! nvv4l2h264enc ! h264parse ! qtmux ! filesink location=/tmp/p_hw3.mp4", sz);
    tryGst("hw4(bitrate)",
        "appsrc ! videoconvert ! video/x-raw,format=NV12 ! nvvidconv ! nvv4l2h264enc bitrate=8000000 ! h264parse ! qtmux ! filesink location=/tmp/p_hw4.mp4", sz);

    readback("/tmp/probe_mp4v.mp4");
    readback("/tmp/probe_avc1.mp4");
    readback("/tmp/probe_gst_sw.mp4");
    return 0;
}
