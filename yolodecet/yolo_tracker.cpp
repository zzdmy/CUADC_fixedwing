// yolo_tracker.cpp
#include "yolo_tracker.h"
#include "../FrameDispatcher.h"

#include <chrono>
#include <iomanip>  // for std::setprecision
#include "../AppLogger.h"
using namespace cv;
using namespace std;

void runInThread(const std::string& modelPath, const std::string& classPath,
	bool runOnGPU, std::atomic<bool>& live) {
	fristyolo(modelPath, classPath, runOnGPU);
	Inference inf(modelPath, cv::Size(640, 640), classPath, runOnGPU);
	AppLogger::get().info("YOLO开始初始化");
	init_pipeline();

	uint64_t last_frame_id = 0;

	while (live.load(std::memory_order_acquire)) {
		auto frame_ptr = frameDispatcher.getFrame();
		if (frame_ptr && !frame_ptr->empty()) {
			uint64_t current_frame_id = frameDispatcher.getFrameCounter();
			if (current_frame_id > last_frame_id) {
				last_frame_id = current_frame_id;

				std::vector<yoloout> output;
				bool dummy = true;
				runObjectTracking(*frame_ptr, inf, output, dummy);

				push_yolo(std::move(output)); // �� move ����
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	signal_stop();
}

// ����ļ��Ƿ����
bool fileExists(const string& filename) {
	ifstream file(filename);

	return file.good();
}



// ��ʼ��������
void yolo(Inference inf, Mat& frame, vector<yoloout>& getout) {
	vector<Detection> output = inf.runInference(frame);
	// ��ȡ��⵽��Ŀ������
	int detections = output.size();
	// ����ÿһ����⵽��Ŀ��
	for (int i = 0; i < detections; ++i)
	{
		// ��ȡ��ǰĿ�����ϸ��Ϣ
		Detection detection = output[i];
		// Ŀ��߽����Ϣ
		Rect box = detection.box;
		Point center((box.x + box.width / 2), (box.y + box.height / 2));
		Mat result = extractSafeSubRect(frame, box);//�ӻ�����ȡ������Ҫ����쾮������Σ�
		yoloout out{ detection.className,detection.confidence,result ,center,box,detection.class_id };
		getout.push_back(out);//���
	}
}
// ��ȫ����ȡ�Ӿ���
Mat extractSafeSubRect(Mat& workingImage, const Rect& box) {
	Rect rect = box;

	// �������α߽���ȷ������ȫλ�� workingImage �ĳߴ緶Χ��
	if (rect.x < 0) {
		rect.x = 0;
	}
	if (rect.y < 0) {
		rect.y = 0;
	}
	if (rect.br().x > workingImage.cols) {
		rect.width = workingImage.cols - rect.x;
	}
	if (rect.br().y > workingImage.rows) {
		rect.height = workingImage.rows - rect.y;
	}
	if (rect.width <= 0 || rect.height <= 0) {
		return cv::Mat();
	}

	return workingImage(rect);
}

void fristyolo(const std::string& modelPath, const std::string& classPath, bool runOnGPU) {
	AppLogger::get().info("开始初始化yolo模型");
	if (!fileExists(modelPath)) {
		AppLogger::get().error("模型文件不存在: {}", modelPath);
		return;
	}
	if (!fileExists(classPath)) {
		AppLogger::get().error("类别文件不存在: {}", classPath);
		return;
	}

	AppLogger::get().info("开始初始化推理引擎");
}

void runObjectTracking(Mat inframe, Inference inf, vector<yoloout>& output, bool& yolostop) {
	vector<yoloout> getout;//��������
	yolo(inf, inframe, getout);//����
	output = getout;//��������
}

void drawCompassOnImage(cv::Mat& image, float yaw) {

	// ����һЩ����
	const int center_x = image.cols / 2;
	const int center_y = image.rows / 2;
	//const int radius = std::min(center_x, center_y) - 10; // ���̰뾶������һЩ�߾�

	//// ����ԲȦ��ʾ���̱߽�
	//cv::circle(image, cv::Point(center_x, center_y), radius, cv::Scalar(0, 255, 0), 2);

	//yaw = yaw + 90;

	//if (yaw == 0) {
	//	yaw = 0.1;
	//}

	//// ����ָ�򱱷�����
	//float north_x = center_x + radius * cos(yaw * CV_PI / 180.0); // ��yaw�Ӷ�ת��Ϊ����
	//float north_y = center_y - radius * sin(yaw * CV_PI / 180.0); // ע��y�᷽���෴
	//cv::line(image, cv::Point(center_x, center_y), cv::Point(north_x, north_y), cv::Scalar(0, 0, 255), 2);

	//// ��������N��Ǳ���
	//cv::putText(image, "N", cv::Point(north_x, north_y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

	//// ѭ����������ÿ��5�ȵķ����ǺͽǶ���ֵ
	//for (int angle = 0; angle <= 360; angle += 5) {
	//	float radian = (yaw - angle) * CV_PI / 180.0;

	//	// ���㵱ǰ�Ƕ�λ���ϵĵ�����
	//	float x = center_x + (radius - 10) * cos(radian);
	//	float y = center_y - (radius - 10) * sin(radian);

	//	if (angle % 90 == 0) {
	//		std::string direction;
	//		switch (angle) {
	//		case 90: direction = "E"; break;
	//		case 180: direction = "S"; break;
	//		case 270: direction = "W"; break;
	//		default: break; //
	//		}
	//		if (!direction.empty()) {
	//			cv::putText(image, direction, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
	//		}
	//	}
	//	else {
	//		// ���ƽǶ���ֵ
	//		std::ostringstream oss;
	//		oss << angle;
	//		std::string buffer = oss.str();
	//		cv::putText(image, buffer, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
	//	}
	//}

	vector<yoloout> indraw;//

	//  ��������ȡ�������ݣ�����Ϊ nullptr��
	auto yolodata = get_latest_yolo();

	if (yolodata) {
		indraw = *yolodata;//��ȡ�����
	}//�������Ϊ��

	if (indraw.size() > 0) {
		line(image, cv::Point(center_x, center_y), indraw[0].corner, cv::Scalar(0, 255, 0), 2);
	}

	for (const auto& out : indraw) {
		// ����Ŀ��
		rectangle(image, out.box, Scalar(0, 255, 0), 1);
		putText(image, to_string(out.confidence), out.box.tl(), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);
	}
}