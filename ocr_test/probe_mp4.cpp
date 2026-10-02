// ocr_test/probe_mp4.cpp
// 探测机载 OpenCV/GStreamer 能写哪些格式（MP4 候选路径）
#include <cstdio>
#include <opencv2/opencv.hpp>

static void tryWrite(const char* name, const std::string& path, int fourcc, const cv::Size& sz) {
    cv::VideoWriter w(path, fourcc, 30, sz);
    std::printf("%-24s opened=%d\n", name, w.isOpened() ? 1 : 0);
    if (w.isOpened()) {
        cv::Mat f(sz, CV_8UC3, cv::Scalar(0, 0, 200));
        for (int i = 0; i < 30; ++i) w.write(f);
        w.release();
    }
}

static void tryGst(const char* name, const std::string& pipe, const cv::Size& sz) {
    cv::VideoWriter w(pipe, cv::CAP_GSTREAMER, 0, 30, sz);
    std::printf("%-24s opened=%d\n", name, w.isOpened() ? 1 : 0);
    if (w.isOpened()) {
        cv::Mat f(sz, CV_8UC3, cv::Scalar(0, 200, 0));
        for (int i = 0; i < 30; ++i) w.write(f);
        w.release();
    }
}

int main() {
    const cv::Size sz(1920, 1080);
    tryWrite("mp4v .mp4", "/tmp/probe_mp4v.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), sz);
    tryWrite("avc1 .mp4", "/tmp/probe_avc1.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), sz);
    tryWrite("MJPG .avi", "/tmp/probe_mjpg.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), sz);

    tryGst("gst h264 mp4 (nv)",
        "appsrc ! videoconvert ! nvvidconv ! nvv4l2h264enc ! h264parse ! qtmux ! filesink location=/tmp/probe_gst.mp4",
        sz);
    tryGst("gst h264 mp4 (sw)",
        "appsrc ! videoconvert ! x264enc speed-preset=ultrafast ! h264parse ! qtmux ! filesink location=/tmp/probe_gst_sw.mp4",
        sz);
    return 0;
}
