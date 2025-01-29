#pragma once

#include <opencv2/opencv.hpp>
#include <YOLOPoseEstimation.h>
#include <xmemory>
#include <string>
#include <vector>
#include <unordered_map>
#include <Vector3D.h>
namespace MCB
{

	struct CaptureData
	{
		std::string captureBoneName;
		Vector3D captureBonePos;
		Vector3D initializedCaptureBonePos;
		std::vector<CaptureData*> captureChildren;
		CaptureData* parent;

	};

	class Capture
	{
	private:
		std::array<std::string,17> linkBoneNames = { "none","none" ,"none" ,"none" ,"none" ,
													"upper_arm.L","upper_arm.R","forearm.L","forearm.R","hand.L",
													"hand.R" ,"none" ,"none" ,"none" ,"none" ,
													"none","none" };

		std::unordered_map<YOLO_POSE_INDEX,CaptureData> capturedata_;
		cv::VideoCapture capture_;
		cv::Mat img_;
		const std::string& modelPath_ = "Checkpoints/yolo11x-pose.onnx";
		std::unique_ptr<YOLOPoseEstimation> m_YOLOPoseEstimation_;
		const YOLO_POSE_LANDMAKE* land_;
		float mask_threshold_ = 0.5f;
		float conf_threshold_ = 0.30f;
		float iou_threshold_ = 0.45f;
		int conversion_code_ = cv::COLOR_BGR2RGB;
		bool initialized_ = true;
		int index_;
		std::string windowName;
	public:
		float cameradist_ = 1.0f;//メートル単位
		Vector3D cameraPosition_;
		float focalLength_;
		Vector3D screenCenterPos_;

		void Initialize(const int32_t index = 0);
		void Update();
		void SetInitialPose();
		void Finalize();
		CaptureData& GetCaptureData(YOLO_POSE_INDEX key);
	};
}