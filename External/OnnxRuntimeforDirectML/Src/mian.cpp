#include <opencv2/opencv.hpp>
#include <YOLOPoseEstimation.h>

#include<iostream>

int main()
{
	cv::VideoCapture cap(0);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 600);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	if (!cap.isOpened())
	{
		return -1;
	}

	cv::Mat img;

	img = cv::imread("Rairu.jpg");

	const std::string& modelPath = "Checkpoints/yolo11x-pose.onnx";

	float mask_threshold = 0.5f;
	float conf_threshold = 0.30f;
	float iou_threshold = 0.45f;
	int conversion_code = cv::COLOR_BGR2RGB;

	std::unique_ptr<YOLOPoseEstimation>m_YOLOPoseEstimation;

	m_YOLOPoseEstimation.reset(CreateYOLOPoseEstimation());

	m_YOLOPoseEstimation->CameraInitialize(&cap);

	m_YOLOPoseEstimation->ModelInitialize(modelPath.c_str());

	m_YOLOPoseEstimation->Start(true);

	const YOLO_POSE_LANDMAKE* land;

	while (true)
	{
		cv::imshow("title", img);

		land = m_YOLOPoseEstimation->GetLandmakes();

		for (size_t i = 0; i < 17; i++)
		{
			std::cout << i << ':' << " X:" << land[i].x << " Y:" << land[i].y << " Vi:" << land[i].vi << std::endl;
		}

		const int key = cv::waitKey(1);

		system("cls");

		if (key == 'q')
		{
			m_YOLOPoseEstimation->End();

			break;
		}
	}

	cv::destroyWindow("title");

	return 0;
}
