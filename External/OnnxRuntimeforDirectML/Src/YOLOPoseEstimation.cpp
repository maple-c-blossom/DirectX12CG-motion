#include "YOLOPoseEstimation.h"
#include <Vector3.h>
#include <Matrix3x3.h>

#include<filesystem>
#include<array>
#include<vector>
#include<string>
#include<memory>
#include<thread>
#include <mutex>
#include <atomic>

#include "autobackend.h"
#include <opencv2/opencv.hpp>

#include "augment.h"
#include <dml_provider_factory.h>
#include <onnxruntime_cxx_api.h>

#include "constants.h"
#include "common.h"
#include "onnx_model_base.h"
using namespace MCBM;

struct CaptureData
{
	std::string captureBoneName;
	MCBM::Vector3 captureBonePos;
	MCBM::Vector3 initializedCaptureBonePos;
	std::vector<CaptureData*> captureChildren;
	CaptureData* parent;

};

struct Vec2
{
	int32_t x;
	int32_t y;
};

struct Vec2F
{
	float x;
	float y;
};

Vec2F Midpoint(const Vec2F& p1, const Vec2F& p2)
{
	Vec2F mid;
	mid.x = (p1.x + p2.x) / 2.0;
	mid.y = (p1.y + p2.y) / 2.0;
	return mid;
}

Vec2F Translate(const Vec2F& p, const Vec2F& origin)
{
	Vec2F translated_point;
	translated_point.x = p.x - origin.x;
	translated_point.y = p.y - origin.y;
	return translated_point;
}

class YOLOPoseEstimationImp : public YOLOPoseEstimation
{
public:
	YOLOPoseEstimationImp();
	~YOLOPoseEstimationImp();

public:

	void CameraInitialize(void* cam,float cametaDist) override;

	void ModelInitialize(const char* modelPath, float mask_threshold, float conf_threshold, float iou_threshold, ONNXP_ROVIDERS provider) override;

	void Start(bool isDraw) override;

	std::unordered_map <YOLO_POSE_INDEX,Vector3>* const GetLandmakes() override;

	void End() override;

	void Update();

private:
	void GetFrame(int32_t index);

	void _Draw(cv::Mat& image,int32_t index);

	bool _AllComplete();

	void _CompleteReset();
	void CalclateFinalCaptureData();

	void computeRay(const cv::Mat& R,const cv::Mat& t,const cv::Point2f& undistNorm,Vector3& camCenterW,Vector3& dirW);
	void CalclateFinalCaptureDataFromCalibrateData();
	void AddCameraData(std::string filepath);
private:

	const std::array<Vec2, 19> m_skeleton = { {{16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13}, {6, 7},{6, 8}, {7, 9}, {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7} } };
	const std::array<cv::Scalar, 4> m_posePalette = { cv::Scalar(255, 128, 0), cv::Scalar(255, 51, 255), cv::Scalar(51, 153, 255),cv::Scalar(0, 255, 0) };
	const std::array<int32_t, 19> m_limbColorIndices = { 2, 2, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 3, 3, 3, 3, 3, 3, 3 };
	const std::array<int32_t, 17> m_kptColorIndices = { 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2 };

	const std::array<std::string, 3> m_provider = { {{"cpu"},{"cuda"},{"directml"}} };

	std::vector<std::array<YOLO_POSE_LANDMAKE, 17>> m_baseLandmakes;
	std::vector<std::array<YOLO_POSE_LANDMAKE, 17>> m_landmakes;
	std::vector<cv::VideoCapture*> m_pCams;
	std::vector<cv::Mat> frames;
	std::vector<bool> m_getFrameComplete;
	std::vector<std::thread> m_frameThread;
	

	float m_maskThreshold;
	float m_confhhreshold;
	float m_iouThreshold;
	int32_t m_conversionCode = cv::COLOR_BGR2RGB;

	std::unique_ptr<AutoBackendOnnx> m_pModel;
	std::string m_modelPath;
	const std::string m_onnxLogid = "yolov8_inference2";
	std::vector<std::string> winName = { "win1","win2","win3","win4"};
	bool m_isDraw;
	bool m_canDraw;
	bool m_emission;
	std::atomic<bool> isRunning;
	std::thread th;
	std::mutex value_mutex;

	std::array<std::unordered_map<YOLO_POSE_INDEX,CaptureData>,4> capturedata_;
	std::unordered_map <YOLO_POSE_INDEX,Vector3> finalCaptureData_;

	std::vector<float> cameradist = {1.f,1.f,1.f,1.f};//メートル単位
	std::vector<Vector3> cameraPosition_;
	std::vector<float> focalLength_ = { 581.818f,581.818f,581.818f,581.818f };
	Vector3 screenCenterPos_ = { CAMERA_WITH / 2,CAMERA_HIGHT / 2,0 };


	// 歪み係数 (k1, k2, p1, p2, k3...) の例
	std::vector<cv::Mat> distCoeffs;
	std::vector<cv::Mat> K;
};

YOLOPoseEstimation* CreateYOLOPoseEstimation()
{
	static YOLOPoseEstimation* result;

	if (!result)
	{
		result = new YOLOPoseEstimationImp();
	}

	return result;
}

YOLOPoseEstimationImp::YOLOPoseEstimationImp()
{
}

YOLOPoseEstimationImp::~YOLOPoseEstimationImp()
{

}

void YOLOPoseEstimationImp::CameraInitialize(void* cam,float cameraDist)
{
	m_pCams.push_back((cv::VideoCapture*)cam);
	cameradist.push_back(cameraDist);
}

void YOLOPoseEstimationImp::ModelInitialize(const char* modelPath, float maskThreshold, float confThreshold, float iouThreshold, ONNXP_ROVIDERS provider)
{
	m_modelPath = modelPath;
	m_maskThreshold = maskThreshold;
	m_confhhreshold = confThreshold;
	m_iouThreshold = iouThreshold;

	m_pModel = std::make_unique<AutoBackendOnnx>(m_modelPath.c_str(), m_onnxLogid.c_str(), m_provider[size_t(provider)].c_str());
}

void YOLOPoseEstimationImp::Start(bool isDraw)
{
	m_isDraw = isDraw;
	isRunning = true;
	m_frameThread.resize(m_pCams.size());
	frames.resize(m_pCams.size());
	m_getFrameComplete.resize(m_pCams.size());

	for ( int32_t i = 0; i < (int32_t)YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		finalCaptureData_[ ( YOLO_POSE_INDEX ) i ] = {};
		for ( int32_t i = 0; i < capturedata_.size(); i++ )
		{
			capturedata_[i][ ( YOLO_POSE_INDEX ) i ] = {};
		}
	}

	for ( size_t i = 0; i < m_pCams.size(); i++ )
	{
		m_frameThread[ i ] = std::thread([ this,i ] ()
			{
				this->GetFrame(i);
			});
	}
	th = std::thread([this]()
		{
			this->Update();
		});

	

}

 std::unordered_map <YOLO_POSE_INDEX,Vector3>* const YOLOPoseEstimationImp::GetLandmakes()
{
	return &finalCaptureData_;
}

void YOLOPoseEstimationImp::End()
{
	isRunning = false;

	th.join();
}

void YOLOPoseEstimationImp::Update()
{
	m_canDraw = false;


	while ( isRunning )
	{
		m_emission = false;


		for ( size_t i = 0; i < m_pCams.size(); i++ )
		{
			
			std::vector<YoloResults> objs;

			if ( m_pModel )
			{
				std::lock_guard<std::mutex> lock(value_mutex);
				objs = m_pModel->predict_once(frames[ i ],m_confhhreshold,m_iouThreshold,m_maskThreshold);
			}

			if ( !objs.empty() )
			{
				for ( int j = 0; j < ( int ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; j++ )
				{
					int idx = j * 3;
					m_baseLandmakes[ i ][j].x = objs[ 0 ].keypoints[ idx ];
					m_baseLandmakes[ i ][j].y = objs[ 0 ].keypoints[ idx + 1 ];
					m_baseLandmakes[ i ][j].vi = objs[ 0 ].keypoints[ idx + 2 ];
				}

				Vec2F mid = Midpoint({ m_baseLandmakes[i][ size_t(YOLO_POSE_INDEX::HIP_L) ].x, m_baseLandmakes[i][ size_t(YOLO_POSE_INDEX::HIP_L) ].y },{ m_baseLandmakes[i][ size_t(YOLO_POSE_INDEX::HIP_R) ].x, m_baseLandmakes[i][ size_t(YOLO_POSE_INDEX::HIP_R) ].y });

				std::lock_guard<std::mutex> lock(value_mutex);

				for ( int j = 0; j < ( int ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; j++ )
				{
					Vec2F newPoint = Translate({ m_baseLandmakes[ i ][j].x, m_baseLandmakes[ i ][j].y},mid);

					m_landmakes[ i ][j].x = newPoint.x;
					m_landmakes[ i ][j].y = newPoint.y;
					m_landmakes[ i ][j].vi = m_baseLandmakes[ i ][j].vi;

					capturedata_[ i ][ ( YOLO_POSE_INDEX ) j ].captureBonePos.x = m_landmakes[ i ][ j ].x;
					capturedata_[ i ][ ( YOLO_POSE_INDEX ) j ].captureBonePos.y = m_landmakes[ i ][ j ].y;
					capturedata_[ i ][ ( YOLO_POSE_INDEX ) j ].captureBonePos.z = m_landmakes[ i ][ j ].vi;
				}

				m_canDraw = true;
			}

			//if ( m_isDraw )
			//{
			//	_Draw(frames[ i ],i);
			//}

			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		m_emission = true; 
	}
	//for ( size_t i = 0; i < m_pCams.size(); i++ )
	//{
	//	cv::destroyWindow(winName[ i ].c_str());
	//}

}

void YOLOPoseEstimationImp::GetFrame(int32_t index)
{
	if ( index >= frames.size() && index >= m_pCams.size() )
	{
		return;
	}
	while ( isRunning )
	{
		
		m_pCams[ index ]->read(frames[ index ]);
		m_getFrameComplete[ index ] = true;
		while ( !m_emission && !_AllComplete())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		m_getFrameComplete[ index ] = false;
	} 
}

void YOLOPoseEstimationImp::_Draw(cv::Mat& image,int32_t index)
{
	cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
	cv::Size show_shape = image.size();

	if (m_canDraw)
	{
		for (int i = 0; i < m_baseLandmakes.size(); i++)
		{
			if (m_baseLandmakes[index][i].vi < 0.5)
			{
				continue;
			}

			cv::circle(image, cv::Point(m_baseLandmakes[ index ][i].x, m_baseLandmakes[ index ][i].y), 5, m_posePalette[m_kptColorIndices[i]], -1, cv::LINE_AA);
		}

		for (int i = 0; i < m_skeleton.size(); i++)
		{
			const Vec2& sk = m_skeleton[i];

			int idx1 = sk.x - 1;
			int idx2 = sk.y - 1;

			int idx1_x_pos = idx1;
			int idx2_x_pos = idx2;

			int x1 = static_cast<int>(m_baseLandmakes[ index ][idx1_x_pos].x);
			int y1 = static_cast<int>(m_baseLandmakes[ index ][idx1_x_pos].y);
			int x2 = static_cast<int>(m_baseLandmakes[ index ][idx2_x_pos].x);
			int y2 = static_cast<int>(m_baseLandmakes[ index ][idx2_x_pos].y);

			float conf1 = m_baseLandmakes[ index ][idx1_x_pos].vi;
			float conf2 = m_baseLandmakes[ index ][idx2_x_pos].vi;

			if (conf1 < 0.5 || conf2 < 0.5)
			{
				continue;
			}

			if (x1 % show_shape.width == 0 || y1 % show_shape.height == 0 || x1 < 0 || y1 < 0 ||
				x2 % show_shape.width == 0 || y2 % show_shape.height == 0 || x2 < 0 || y2 < 0)
			{
				continue;
			}

			cv::Scalar color_limb = m_posePalette[m_limbColorIndices[i]];
			cv::line(image, cv::Point(x1, y1), cv::Point(x2, y2), color_limb, 2, cv::LINE_AA);
		}
	}

	cv::imshow(winName[index].c_str(),image);
	CalclateFinalCaptureData();
	cv::waitKey(1);
}

bool YOLOPoseEstimationImp::_AllComplete()
{
	std::lock_guard<std::mutex> lock(value_mutex);
	for ( size_t i = 0; i < m_getFrameComplete.size(); i++ )
	{
		if ( !m_getFrameComplete[ i ] )
		{
			return false;
		}
	}
	return true;
}

void YOLOPoseEstimationImp::_CompleteReset()
{
	for ( size_t i = 0; i < m_getFrameComplete.size(); i++ )
	{
		m_getFrameComplete[ i ] = false;
	}
}


void YOLOPoseEstimationImp::CalclateFinalCaptureData()
{
	Vector3 finalData;
	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		CaptureData frontCamera = capturedata_[ Locate::FRONT ][( YOLO_POSE_INDEX ) i];
		//CaptureData backCamera = capturedata_[ Locate::BACK ][ ( YOLO_POSE_INDEX ) i ];
		CaptureData leftCamera = capturedata_[ Locate::LEFT ][ ( YOLO_POSE_INDEX ) i ];
		CaptureData rightCamera = capturedata_[ Locate::RIGHT ][ ( YOLO_POSE_INDEX ) i ];

		Vector3 tmpF(frontCamera.captureBonePos.x - screenCenterPos_.x
			,frontCamera.captureBonePos.y - screenCenterPos_.y,
			-focalLength_[ Locate::FRONT ]);

		/*Vector3 tmpB(backCamera.captureBonePos.x - screenCenterPos_.x,
			backCamera.captureBonePos.y - screenCenterPos_.y,
			focalLength_[ Locate::BACK ]);*/

		Vector3 tmpL(focalLength_[ Locate::LEFT ],
			leftCamera.captureBonePos.y - screenCenterPos_.y,
			-( leftCamera.captureBonePos.x - screenCenterPos_.x));

		Vector3 tmpR(-focalLength_[ Locate::RIGHT ],
			rightCamera.captureBonePos.y - screenCenterPos_.y,
			+( rightCamera.captureBonePos.x - screenCenterPos_.x ));

		Vector3 dF = tmpF.GetV3Norm();
		//Vector3 dB = tmpB.GetV3Norm();
		Vector3 dL = tmpL.GetV3Norm();
		Vector3 dR = tmpR.GetV3Norm();

		Matrix3x3 Q;
		for ( int i = 0; i < 9; i++ )
		{
			Q.mat[ i ] = 0.0;
		}
		Vector3 C(0.0,0.0,0.0);

		auto accumulate_line = [ & ] (const Vector3& p,const Vector3& d,double w)
			{
				// P = I - d d^T
				Matrix3x3 P = P.ProjectionMatrix(d);
				// Q += w * P
				Matrix3x3 wP = wP.Mat3Scale(P,w);
				Q = Q.Mat3Add(Q,wP);
				// c += w * P * p
				Vector3 Pp = P.Mat3Mulvec(P,p);
				C = C + ( w * Pp );
			};

		// 前カメラ
		accumulate_line(cameraPosition_[ Locate::FRONT ],dF,frontCamera.captureBonePos.z);
		// 後カメラ
		//accumulate_line(cameraPosition_[ Locate::BACK ],dB,backCamera.captureBonePos.z);
		// 左カメラ
		accumulate_line(cameraPosition_[ Locate::LEFT ],dL,leftCamera.captureBonePos.z);
		// 右カメラ
		accumulate_line(cameraPosition_[ Locate::RIGHT ],dR,rightCamera.captureBonePos.z);

		// (6) 連立方程式 Q X = C を解く (Xが最小二乗解)
		Matrix3x3 Qinv;
		bool ok = Q.Invert3x3(Q,Qinv);
		Vector3 X(0,0,0);
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
		finalCaptureData_[ ( YOLO_POSE_INDEX ) i ] = finalData;
	}
}

void YOLOPoseEstimationImp::computeRay(const cv::Mat& R,const cv::Mat& t,const cv::Point2f& undistNorm,Vector3& camCenterW,Vector3& dirW)
{
	// カメラ中心 (world系) = -R^T * t
	cv::Mat Rt = R.t(); // Rの転置
	cv::Mat center = -Rt * t; // (3x1)
	camCenterW = { static_cast<float>(center.at<double>(0)), static_cast< float >( center.at<double>(1)),
		 static_cast< float >( center.at<double>(2)) };

	// カメラ座標系でのベクトル: (x_nd, y_nd, 1)
	// → ワールド座標系へは R^T で回転
	cv::Mat dirCam = ( cv::Mat_<double>(3,1) << undistNorm.x,undistNorm.y,1.0 );
	cv::Mat dirWorld = Rt * dirCam; // (3x1)
	Vector3 dw = {
		 static_cast<float>(dirWorld.at<double>(0)),
		 static_cast<float>(dirWorld.at<double>(1)),
		 static_cast<float>(dirWorld.at<double>(2))
	};
	dw.GetV3Norm();
	dirW = dw;
}


void YOLOPoseEstimationImp::CalclateFinalCaptureDataFromCalibrateData()
{
	//===============================================================
	// 例) 4台のカメラ: それぞれ外部パラメータ (R_i, t_i) が既知とする
	//    OpenCV流にいうと、X_c = R_i * X_w + t_i
	//===============================================================
	// ここでは「front, back, left, right」に相当する適当な例を作る
	// 実際にはキャリブレーションやマーカー計測などで得たR,tを入れる
	//
	// front:  ワールド座標系の前方にZ負方向でカメラを見るイメージ
	// back :  後方にZ正方向でカメラを見るイメージ
	// left :  X負方向
	// right:  X正方向
	// (単にダミーで回転行列を用意)
	//---------------------------------------------------------------
	// （注意）ここでは「R_i, t_i は world->camera」の回転・並進行列
	//          たとえば front は「Z軸正方向をカメラ-Zで見る」など
	//          実際の値は環境に依存します。
	//---------------------------------------------------------------
	std::vector<cv::Mat> Rvecs(4),tvecs(4);

	// front (原点から+Z方向=1.0m、カメラは -Z を向く)
	// → つまりワールド座標でカメラの位置 (0,0,+1)
	//    カメラ座標の Z軸がワールド座標系の -Z を向くには、回転行列は
	//    「180度回転」around X軸(かつY軸反転しないように調整)など
	{
		cv::Mat Rf = cv::Mat::eye(3,3,CV_64F);
		// 例: 回転行列で z->-z, すなわち y->-y, など
		//     ここでは (X, Y, Z)->(X, -Y, -Z) の行列を作成する
		Rf.at<double>(1,1) = -cameradist[FRONT];
		Rf.at<double>(2,2) = -cameradist[FRONT];
		Rvecs[ 0 ] = Rf;
		// t = (0,0,1) in "camera = R*world + t" => 
		// → world原点(0,0,0)がカメラ座標系でどう見えるか? 
		//   ここではダミーで(0,0,1)
		tvecs[ 0 ] = ( cv::Mat_<double>(3,1) << 0,0,cameradist[FRONT] );
	}
	// back (原点から-Z方向=1.0m、カメラは +Z を向く)
	{
		cv::Mat Rb = cv::Mat::eye(3,3,CV_64F);
		// 例: (X,Y,Z)->(X,Y,Z) のまま
		//     カメラ座標系ZとワールドZが同じ向き
		Rvecs[ 1 ] = Rb;
		// t = (0,0,-1)
		tvecs[ 1 ] = ( cv::Mat_<double>(3,1) << 0,0,-cameradist[ BACK ] );
	}
	// left (原点から -X=1.0m、 カメラは +X を向く)
	{
		// 例: (X,Y,Z)->(Z,Y,-X) のような90度回転(簡易例)
		cv::Mat Rl = ( cv::Mat_<double>(3,3) <<
			0,0,-cameradist[ LEFT ],
			0,cameradist[ LEFT ],0,
			cameradist[ LEFT ],0,0
		);
		Rvecs[ 2 ] = Rl;
		tvecs[ 2 ] = ( cv::Mat_<double>(3,1) << -cameradist[ LEFT ],0,0 );
	}
	// right (原点から +X=1.0m、 カメラは -X を向く)
	{
		// 例: (X,Y,Z)->(-Z,Y,X)
		cv::Mat Rr = ( cv::Mat_<double>(3,3) <<
			0,0,cameradist[ RIGHT ],
			0,cameradist[ RIGHT ],0,
		   -cameradist[ RIGHT ],0,0
		);
		Rvecs[ 3 ] = Rr;
		tvecs[ 3 ] = ( cv::Mat_<double>(3,1) << +cameradist[ RIGHT ],0,0 );
	}
	Vector3 finalData;
	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
	//===============================================================
	// 例) YOLO等から得られた4台分の (u,v) と 信頼度 c
	//===============================================================
	//   ここではテスト値を適当に設定
		std::vector<cv::Point2f> imagePts{
			{capturedata_[ Locate::FRONT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.x,
			capturedata_[ Locate::FRONT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.y}, // front

			{capturedata_[ Locate::BACK ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.x,
			capturedata_[ Locate::BACK ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.y}, // back

			{capturedata_[ Locate::LEFT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.x,
			capturedata_[ Locate::LEFT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.y}, // left

			{capturedata_[ Locate::RIGHT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.x,
			capturedata_[ Locate::RIGHT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.y}, // right
		};
		std::vector<double> confidences{ capturedata_[ Locate::FRONT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.z
			, capturedata_[ Locate::BACK ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.z
			, capturedata_[ Locate::LEFT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.z
			, capturedata_[ Locate::RIGHT ][ ( YOLO_POSE_INDEX ) i ].captureBonePos.z };

		//===============================================================
		// (1) 歪み補正 & 正規化座標化
		//     OpenCVの undistortPoints() を利用
		//===============================================================
		//   undistortPoints()の出力は 「(x_nd, y_nd)」(Z=1相当) なので
		//   これを各カメラの外部パラメータに適用してレイを算出する
		//---------------------------------------------------------------
		//   注意: undistortPoints() は複数点をまとめて処理できる。
		//---------------------------------------------------------------
		std::vector<cv::Point2f> undistNormPoints;
		{
			// 入力をvectorに (今回は4点だけ)
			std::vector<cv::Point2f> inputPts = imagePts;

			// alpha=0の新しいカメラ行列(ここではKをそのまま使うでもOK)
			std::vector< cv::Mat> newK = K;
			// 歪み補正して正規化座標取得
			cv::undistortPoints(inputPts,undistNormPoints,K,distCoeffs,cv::noArray(),newK);
			// 出力 undistNormPoints[i] = (x_nd, y_nd)
		}

		//===============================================================
		// (2) 各カメラでレイをワールド座標系に表現
		//     方向ベクトル(単位) dW[i], カメラ中心 cW[i]
		//===============================================================
		std::vector<Vector3> cW(4),dW(4); // cameraCenterWorld, directionWorld
		for ( int i = 0; i < 4; i++ )
		{
			computeRay(Rvecs[ i ],tvecs[ i ],undistNormPoints[ i ],cW[ i ],dW[ i ]);
		}

		//===============================================================
		// (3) 重み付き最小二乗 (Weighted LS) の計算
		//     Q = Σ_i w_i (I - d_i d_i^T)
		//     c = Σ_i w_i (I - d_i d_i^T) cW[i]
		//     Q X = c  を解く
		//===============================================================
		Matrix3x3 Q;
		for ( int k = 0; k < 9; k++ )
		{
			Q.mat[ k ] = 0.0;
		}
		Vector3 C{ 0,0,0 };

		for ( int i = 0; i < 4; i++ )
		{
			double w = confidences[ i ]; // カメラiの信頼度
			Vector3 di = dW[ i ];
			// I - d_i d_i^T
			Matrix3x3 Pi = Pi.ProjectionMatrix(di);
			// w*Pi
			Matrix3x3 wPi = Pi.Mat3Scale(Pi,w);
			// Q += wPi
			Q = Q.Mat3Add(Q,wPi);
			// c += wPi * cW[i]
			Vector3 tmp =wPi.Mat3Mulvec(wPi,cW[ i ]);
			C = C + tmp;
		}

		// Qを逆行列化して X= Q^-1 * c
		Matrix3x3 Qinv;
		Vector3 X{ 0,0,0 };
		if ( Q.Invert3x3(Q,Qinv) )
		{
			Vector3 sol = Qinv.Mat3Mulvec(Qinv,C);
			X = sol;
		}
		else
		{
			std::cerr << "Warning: Q is singular!\n";
		}

		finalData = X;

	}
	return;
}

void YOLOPoseEstimationImp::AddCameraData(std::string filepath)
{
	/*数値内訳
	//===============================================================
	 カメラ内部パラメータ (fx, fy, cx, cy) & 歪み係数が既知
	   ここでは1台のカメラについて定義し、4台とも同じ値と仮定
	//===============================================================
	double fx=1000.0, fy=1000.0, cx=640.0, cy=360.0;
	// 歪み係数 (k1, k2, p1, p2, k3...) の例
	cv::Mat distCoeffs = (cv::Mat_<double>(1,5) <<
							-0.10, 0.05, 0.001, 0.002, 0.0 // 例
						 );
	cv::Mat K = (cv::Mat_<double>(3,3) <<
					fx,  0, cx,
					 0, fy, cy,
					 0,  0,  1);
	*/

	cv::Mat tempCameraMat;//ファイルから読み込み:カメラ行列
	cv::Mat tempCameraDistCoeffs;//読み込み:歪み係数

	distCoeffs.push_back(tempCameraDistCoeffs);
	K.push_back(tempCameraMat);
}
