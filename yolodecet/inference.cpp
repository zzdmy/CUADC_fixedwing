#include "inference.h"
#include "../AppLogger.h"

Inference::Inference(const std::string& onnxModelPath, const cv::Size& modelInputShape, const std::string& classesTxtFile, const bool& runWithCuda)
{
	modelPath = onnxModelPath;
	modelShape = modelInputShape;
	classesPath = classesTxtFile;
	cudaEnabled = runWithCuda;

	loadOnnxNetwork();
	loadClassesFromFile(); //读取名称文件
}

//std::vector<Detection> Inference::runInference(const cv::Mat& input)
//{
//    cv::Mat modelInput = input;
//    if (letterBoxForSquare && modelShape.width == modelShape.height)
//        modelInput = formatToSquare(modelInput);
//
//    // 确保模型输入尺寸正确
//    cv::resize(modelInput, modelInput, modelShape, 0, 0, cv::INTER_LINEAR);
//
//    cv::Mat blob;
//    cv::dnn::blobFromImage(modelInput, blob, 1.0 / 255.0, modelShape, cv::Scalar(), true, false);
//    net.setInput(blob);
//
//    // 手动指定输出层名称
//    std::vector<cv::String> outputNames = { "output0" };
//
//    std::vector<cv::Mat> outputs;
//    try
//    {
//        net.forward(outputs, outputNames);
//    }
//    catch (const cv::Exception& e)
//    {
//        std::cerr << "Error during forward pass: " << e.what() << std::endl;
//        std::cerr << "Model path: " << modelPath << std::endl;
//        std::cerr << "Input shape: " << modelInput.size() << std::endl;
//        std::cerr << "Output names: ";
//        for (const auto& name : outputNames)
//        {
//            std::cerr << name << " ";
//        }
//        std::cerr << std::endl;
//        exit(EXIT_FAILURE);
//    }
//
//    // 继续处理输出
//    int rows = outputs[0].size[1]; // 5 (x, y, w, h, conf)
//    int num_boxes = outputs[0].size[2]; // 6300
//
//    float* data = (float*)outputs[0].data;
//
//    float x_factor = modelInput.cols / modelShape.width;
//    float y_factor = modelInput.rows / modelShape.height;
//
//    std::vector<int> class_ids;
//    std::vector<float> confidences;
//    std::vector<cv::Rect> boxes;
//
//    for (int i = 0; i < num_boxes; ++i)
//    {
//        float confidence = data[4]; // 置信度分数
//        if (confidence >= modelConfidenceThreshold)
//        {
//            float x = data[0];
//            float y = data[1];
//            float w = data[2];
//            float h = data[3];
//
//            int left = int((x - 0.5 * w) * x_factor);
//            int top = int((y - 0.5 * h) * y_factor);
//
//            int width = int(w * x_factor);
//            int height = int(h * y_factor);
//
//            boxes.push_back(cv::Rect(left, top, width, height));
//            confidences.push_back(confidence);
//            class_ids.push_back(0); // 由于只有一个类别'tj'，类别ID固定为0
//        }
//
//        data += rows; // 移动到下一个检测框
//    }
//
//    std::vector<int> nms_result;
//    cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);
//
//    std::vector<Detection> detections{};
//    for (unsigned long i = 0; i < nms_result.size(); ++i)
//    {
//        int idx = nms_result[i];
//
//        Detection result;
//        result.class_id = class_ids[idx];
//        result.confidence = confidences[idx];
//
//        std::random_device rd;
//        std::mt19937 gen(rd());
//        std::uniform_int_distribution<int> dis(100, 255);
//        result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));
//
//        result.className = classes[result.class_id];
//        result.box = boxes[idx];
//
//        detections.push_back(result);
//    }
//
//    return detections;
//}

std::vector<Detection> Inference::runInference(const cv::Mat& input)
{
	cv::Mat modelInput = input;
	if (letterBoxForSquare && modelShape.width == modelShape.height)
		modelInput = formatToSquare(modelInput);

	cv::Mat blob;
	cv::dnn::blobFromImage(modelInput, blob, 1.0 / 255.0, modelShape, cv::Scalar(), true, false);
	net.setInput(blob);//设置输入

	//手动指定输出层名称
	std::vector<cv::String> outputNames = { "output0" };

	std::vector<cv::Mat> outputs;
	try
	{
		net.forward(outputs, outputNames);//获取输出
	}
	catch (const cv::Exception& e)
	{
		AppLogger::get().error("Error during forward pass: {}", e.what());
		AppLogger::get().error("Model path: {}", modelPath);
				AppLogger::get().error("Input shape: {} x {}", modelInput.size().width, modelInput.size().height);
		{
			std::string namesStr;
			for (const auto& name : outputNames)
			{
				namesStr += name + " ";
			}
			AppLogger::get().error("Output names: {}", namesStr);
		}
		exit(EXIT_FAILURE);
	}

	int rows = outputs[0].size[1];
	int dimensions = outputs[0].size[2];

	bool yolov8 = false;

	// yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])
	if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
	{
		yolov8 = true;
		rows = outputs[0].size[2];
		dimensions = outputs[0].size[1];

		outputs[0] = outputs[0].reshape(1, dimensions);
		cv::transpose(outputs[0], outputs[0]);
	}
	float* data = (float*)outputs[0].data;

	float x_factor = modelInput.cols / modelShape.width;
	float y_factor = modelInput.rows / modelShape.height;

	std::vector<int> class_ids;
	std::vector<float> confidences;
	std::vector<cv::Rect> boxes;

	for (int i = 0; i < rows; ++i)
	{
		if (yolov8)
		{
			float* classes_scores = data + 4;

			cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
			cv::Point class_id;
			double maxClassScore;

			minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);//

			if (maxClassScore > modelScoreThreshold)
			{
				confidences.push_back(maxClassScore);
				class_ids.push_back(class_id.x);

				float x = data[0];
				float y = data[1];
				float w = data[2];
				float h = data[3];

				int left = int((x - 0.5 * w) * x_factor);
				int top = int((y - 0.5 * h) * y_factor);

				int width = int(w * x_factor);
				int height = int(h * y_factor);

				boxes.push_back(cv::Rect(left, top, width, height));
			}
		}

		data += dimensions;
	}

	std::vector<int> nms_result;
	cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

	std::vector<Detection> detections{};
	for (unsigned long i = 0; i < nms_result.size(); ++i)
	{
		int idx = nms_result[i];

		Detection result;
		result.class_id = class_ids[idx];
		result.confidence = confidences[idx];

		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_int_distribution<int> dis(100, 255);
		result.color = cv::Scalar(dis(gen),
			dis(gen),
			dis(gen));

		result.className = classes[result.class_id];
		result.box = boxes[idx];

		detections.push_back(result);
	}

	return detections;
}

void Inference::loadOnnxNetwork()
{
	net = cv::dnn::readNetFromONNX(modelPath);
	if (net.empty())
	{
		AppLogger::get().error("Failed to load ONNX model: {}", modelPath);
		exit(EXIT_FAILURE);
	}

	if (cudaEnabled)
	{
		AppLogger::get().info("Running on CUDA");
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
	}
	else
	{
		AppLogger::get().info("Running on CPU");
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
	}
	net.enableFusion(false);  // 禁用层融合
}

void Inference::loadClassesFromFile()
{
	std::ifstream inputFile(classesPath);

	if (inputFile.is_open())
	{
		std::string classLine;
		while (std::getline(inputFile, classLine))

			classes.push_back(classLine);
		inputFile.close();
	}
}

cv::Mat Inference::formatToSquare(const cv::Mat& source)
{
	int col = source.cols;
	int row = source.rows;
	int _max = MAX(col, row);
	cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
	source.copyTo(result(cv::Rect(0, 0, col, row)));
	return result;
}