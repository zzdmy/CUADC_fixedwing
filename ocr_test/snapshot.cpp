// ocr_test/snapshot.cpp
//
// 机载相机抓一帧存图（用于 OCR 真图验证，不依赖主程序/飞控）。
//
// 用法:
//   snapshot <out.jpg> [x y w h]
//   可选 x y w h：抓图后先裁剪再保存（模拟检测框裁剪）。
#include <cstdio>
#include <cstdlib>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::printf("usage: snapshot <out.jpg> [x y w h]\n");
        return 2;
    }

    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::printf("[snapshot] open camera 0 failed\n");
        return 1;
    }
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FPS, 60);
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);   // 开启相机自动曝光（与主程序 ae.enable=false 时一致）

    cv::Mat frame, last;
    // 长预热约 3 秒：让自动曝光/白平衡收敛（15 帧太快，曾抓到全黑图）
    for (int i = 0; i < 90; ++i) {
        cap >> frame;
        if (!frame.empty()) last = frame.clone();
    }
    const int expVal = static_cast<int>(cap.get(cv::CAP_PROP_EXPOSURE));
    cap.release();

    if (last.empty()) {
        std::printf("[snapshot] no frame captured\n");
        return 1;
    }

    if (argc >= 6) {
        const int x = std::atoi(argv[2]);
        const int y = std::atoi(argv[3]);
        const int w = std::atoi(argv[4]);
        const int h = std::atoi(argv[5]);
        cv::Rect r(x, y, w, h);
        r &= cv::Rect(0, 0, last.cols, last.rows);
        if (r.area() > 0) last = last(r).clone();
    }

    if (!cv::imwrite(argv[1], last)) {
        std::printf("[snapshot] imwrite failed: %s\n", argv[1]);
        return 1;
    }
    const double meanv = cv::mean(last)[0];
    std::printf("[snapshot] saved %s (%dx%d) exposure=%d mean=%.1f\n",
        argv[1], last.cols, last.rows, expVal, meanv);
    return 0;
}
