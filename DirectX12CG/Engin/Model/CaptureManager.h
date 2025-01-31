#pragma once

#include "Capture.h"
#include <array>
#include <memory>
#include <iostream>
#include <vector>
#include <cmath>
#include <YOLOPoseEstimation.h>

namespace MCB
{

	class CaptureManager
	{

	private:
		std::array<Capture,3> capdatas;
		std::unordered_map<YOLO_POSE_INDEX,CaptureData> finalCaptureData_;
		std::unique_ptr<YOLOPoseEstimation> m_YOLOPoseEstimation_;
		const std::string& modelPath_ = "Checkpoints/yolo11x-pose.onnx";
	public:
		void Initialize();
		void Update();
		void CalclateFinalCaptureData();
		void InitializePose();
		void Finalize();
		CaptureData& GetCaptureData(YOLO_POSE_INDEX key);
		CaptureData& GetLocateCaptureData(YOLO_POSE_INDEX key,Locate locate);

	};
}
