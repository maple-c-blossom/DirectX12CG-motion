#include "CaptureManager.h"
#include <MCBMatrix3x3.h>

using namespace MCB;

void CaptureManager::Initialize()
{
	for ( int32_t i = 0; i < 4; i++ )
	{
		capdatas[ i ].Initialize(static_cast< int32_t >( i ));
	}

	capdatas[ Locate::FRONT ].cameraPosition_ = { 0.0f,0.0f,capdatas[ Locate::FRONT ].cameradist_ };
	capdatas[ Locate::BACK ].cameraPosition_ = { 0.0f,0.0f,-capdatas[ Locate::BACK ].cameradist_ };
	capdatas[ Locate::LEFT ].cameraPosition_ = { -capdatas[ Locate::LEFT ].cameradist_,0.0f,0.0f };
	capdatas[ Locate::RIGHT ].cameraPosition_ = { capdatas[ Locate::RIGHT ].cameradist_,0.0f,0.0f };
}

void CaptureManager::Update()
{
	for ( size_t i = 0; i < 4; i++ )
	{
		capdatas[ i ].Update();
	}
	CalclateFinalCaptureData();
}

void CaptureManager::CalclateFinalCaptureData()
{
	Vector3D finalData;
	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		CaptureData frontCamera = capdatas[ Locate::FRONT ].GetCaptureData(( YOLO_POSE_INDEX ) i);
		CaptureData backCamera = capdatas[ Locate::BACK ].GetCaptureData(( YOLO_POSE_INDEX ) i);
		CaptureData leftCamera = capdatas[ Locate::LEFT ].GetCaptureData(( YOLO_POSE_INDEX ) i);
		CaptureData rightCamera = capdatas[ Locate::RIGHT ].GetCaptureData(( YOLO_POSE_INDEX ) i);

		Vector3D tmpF(frontCamera.captureBonePos.vec_.x_ - capdatas[ Locate::FRONT ].screenCenterPos_.vec_.x_
			,frontCamera.captureBonePos.vec_.y_ - capdatas[ Locate::FRONT ].screenCenterPos_.vec_.y_,
			-capdatas[ Locate::FRONT ].focalLength_);

		Vector3D tmpB(backCamera.captureBonePos.vec_.x_ - capdatas[ Locate::BACK ].screenCenterPos_.vec_.x_,
			backCamera.captureBonePos.vec_.y_ - capdatas[ Locate::BACK ].screenCenterPos_.vec_.y_,
			capdatas[ Locate::BACK ].focalLength_);

		Vector3D tmpL(capdatas[ Locate::LEFT ].focalLength_,
			leftCamera.captureBonePos.vec_.y_ - capdatas[ Locate::LEFT ].screenCenterPos_.vec_.y_,
			-( leftCamera.captureBonePos.vec_.x_ - capdatas[ Locate::LEFT ].screenCenterPos_.vec_.x_ ));

		Vector3D tmpR(-capdatas[ Locate::RIGHT ].focalLength_,
			rightCamera.captureBonePos.vec_.y_ - capdatas[ Locate::RIGHT ].screenCenterPos_.vec_.y_,
			+( rightCamera.captureBonePos.vec_.x_ - capdatas[ Locate::RIGHT ].screenCenterPos_.vec_.x_ ));

		Vector3D dF = tmpF.GetV3Norm();
		Vector3D dB = tmpB.GetV3Norm();
		Vector3D dL = tmpL.GetV3Norm();
		Vector3D dR = tmpR.GetV3Norm();

		Matrix3x3 Q;
		for ( int i = 0; i < 9; i++ )
		{
			Q.mat[ i ] = 0.0;
		}
		Vector3D C(0.0,0.0,0.0);

		auto accumulate_line = [ & ] (const Vector3D& p,const Vector3D& d,double w)
			{
				// P = I - d d^T
				Matrix3x3 P = P.ProjectionMatrix(d);
				// Q += w * P
				Matrix3x3 wP = wP.Mat3Scale(P,w);
				Q = Q.Mat3Add(Q,wP);
				// c += w * P * p
				Vector3D Pp = P.Mat3Mulvec(P,p);
				C = C + ( w * Pp );
			};

		// 前カメラ
		accumulate_line(capdatas[ Locate::FRONT ].cameraPosition_,dF,frontCamera.captureBonePos.vec_.z_);
		// 後カメラ
		accumulate_line(capdatas[ Locate::BACK ].cameraPosition_,dB,backCamera.captureBonePos.vec_.z_);
		// 左カメラ
		accumulate_line(capdatas[ Locate::LEFT ].cameraPosition_,dL,leftCamera.captureBonePos.vec_.z_);
		// 右カメラ
		accumulate_line(capdatas[ Locate::RIGHT ].cameraPosition_,dR,rightCamera.captureBonePos.vec_.z_);

		// (6) 連立方程式 Q X = C を解く (Xが最小二乗解)
		Matrix3x3 Qinv;
		bool ok = Q.Invert3x3(Q,Qinv);
		Vector3D X(0,0,0);
		if ( ok )
		{
			X = Q.Mat3Mulvec(Qinv,C);
		}
		else
		{
			// Qが特異 → 全部平行などの場合。
			// ここでは簡単に(0,0,0)を返す
			std::cerr << "警告: 行列が特異です。解けませんでした。\n";
		}
		finalData = X;
		finalCaptureData_[ ( YOLO_POSE_INDEX ) i ].captureBonePos = finalData;
	}
}

void CaptureManager::InitializePose()
{
	for ( int32_t i = 0; i < 4; i++ )
	{
		capdatas[ i ].Update();
	}
	CalclateFinalCaptureData();

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

