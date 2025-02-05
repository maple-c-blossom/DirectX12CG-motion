#include "CaptureManager.h"
#include <MCBMatrix3x3.h>

using namespace MCB;

void CaptureManager::Initialize()
{
	m_YOLOPoseEstimation_.reset(CreateYOLOPoseEstimation());

	m_YOLOPoseEstimation_->ModelInitialize(modelPath_.c_str());


	for ( int32_t i = 0; i < 3; i++ )
	{
		capdatas[ i ].SetYOLOEstimation(m_YOLOPoseEstimation_.get());
		capdatas[ i ].Initialize(static_cast< int32_t >( i+1 ));
	}

	m_YOLOPoseEstimation_->Start(true);

	capdatas[ Locate::FRONT ].cameraPosition_ = { 0.0f,0.0f,capdatas[ Locate::FRONT ].cameradist_ };
	//capdatas[ Locate::BACK ].cameraPosition_ = { 0.0f,0.0f,-capdatas[ Locate::BACK ].cameradist_ };
	capdatas[ Locate::LEFT ].cameraPosition_ = { -capdatas[ Locate::LEFT ].cameradist_,0.0f,0.0f };
	capdatas[ Locate::RIGHT ].cameraPosition_ = { capdatas[ Locate::RIGHT ].cameradist_,0.0f,0.0f };
}

void CaptureManager::Update()
{
	for ( size_t i = 0; i < 3; i++ )
	{
		capdatas[ i ].Update();
	}

}


void CaptureManager::InitializePose()
{
	for ( int32_t i = 0; i < 3; i++ )
	{
		capdatas[ i ].Update();
	}


	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		finalCaptureData_[ ( YOLO_POSE_INDEX ) i ].initializedCaptureBonePos
			= finalCaptureData_[ ( YOLO_POSE_INDEX ) i ].captureBonePos;
	}


}

void MCB::CaptureManager::Finalize()
{

	for ( int32_t i = 0; i < 4; i++ )
	{
		capdatas[ i ].Finalize();
	}
}

CaptureData& CaptureManager::GetCaptureData(YOLO_POSE_INDEX key)
{
	return finalCaptureData_[ key ];
}

CaptureData& MCB::CaptureManager::GetLocateCaptureData(YOLO_POSE_INDEX key,Locate locate)
{
	return capdatas[ (int32_t)locate ].GetCaptureData(key);
}

