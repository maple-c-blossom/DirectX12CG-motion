#include <Capture.h>
#include <Util.h>

using namespace MCB;
void MCB::Capture::Initialize(const int32_t index)
{
	index_ = index;
	cv::VideoCapture cap;
	capture_ = std::move(cap);
	capture_ = cv::VideoCapture(index,cv::CAP_DSHOW);
	capture_.set(cv::CAP_PROP_FRAME_WIDTH,1280);
	capture_.set(cv::CAP_PROP_FRAME_HEIGHT,720);
	capture_.set(cv::CAP_PROP_FPS,30);
	capture_.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('H','2','6','4'));



	if ( !capture_.isOpened() )
	{
		initialized_ = false;
		return;
	}

	img_ = cv::imread("Resources\\Rairu.jpg");

	if ( img_.empty() )
	{
		initialized_ = false;
		return;
	}

	m_YOLOPoseEstimation_->CameraInitialize(&capture_,1.0f);


	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		CaptureData datatemp;
		capturedata_[ ( YOLO_POSE_INDEX ) i ] = datatemp;
	}
	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		CaptureData* temp = &capturedata_[ ( YOLO_POSE_INDEX ) i ];
		temp->captureBoneName = linkBoneNames[ i ];
		switch ( ( YOLO_POSE_INDEX ) i )
		{
		case YOLO_POSE_INDEX::NOSE:

			break;
		case YOLO_POSE_INDEX::EYE_L:

			break;
		case YOLO_POSE_INDEX::EYE_R:

			break;
		case YOLO_POSE_INDEX::EAR_L:

			break;
		case YOLO_POSE_INDEX::EARR:

			break;
		case YOLO_POSE_INDEX::SHOULDER_L:
			temp->captureChildren.push_back(&capturedata_[ YOLO_POSE_INDEX::ELBOW_L ]);
			break;
		case YOLO_POSE_INDEX::SHOULDER_R:
			temp->captureChildren.push_back(&capturedata_[ YOLO_POSE_INDEX::ELBOW_R ]);
			break;
		case YOLO_POSE_INDEX::ELBOW_L:
			temp->captureChildren.push_back(&capturedata_[ YOLO_POSE_INDEX::WRIST_L ]);
			break;
		case YOLO_POSE_INDEX::ELBOW_R:
			temp->captureChildren.push_back(&capturedata_[ YOLO_POSE_INDEX::WRIST_R ]);
			break;
		case YOLO_POSE_INDEX::WRIST_L:

			break;
		case YOLO_POSE_INDEX::WRIST_R:

			break;
		case YOLO_POSE_INDEX::HIP_L:

			break;
		case YOLO_POSE_INDEX::HIP_R:

			break;
		case YOLO_POSE_INDEX::KNEE_L:

			break;
		case YOLO_POSE_INDEX::KNEE_R:

			break;
		case YOLO_POSE_INDEX::ANKLE_L:

			break;
		case YOLO_POSE_INDEX::ANKLE_R:

			break;
		default:
			break;
		}
	}

	//windowName = "run" + std::to_string(index_);

}

void MCB::Capture::Update()
{
	//cv::imshow(windowName.c_str(),img_);

	land_ = m_YOLOPoseEstimation_->GetLandmakes();
	//skelton構成
	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		capturedata_[ ( YOLO_POSE_INDEX ) i ].captureBonePos = { land_->operator[]( (YOLO_POSE_INDEX) i).x,
			land_->operator[](( YOLO_POSE_INDEX ) i).y,land_->operator[](( YOLO_POSE_INDEX ) i).z };
	}

}

void MCB::Capture::SetInitialPose()
{
	//land_ = m_YOLOPoseEstimation_->GetLandmakes();

	//skelton構成
	for ( int32_t i = 0; i < ( int32_t ) YOLO_POSE_INDEX::YOLO_POSE_INDEX_MAX; i++ )
	{
		capturedata_[ ( YOLO_POSE_INDEX ) i ].initializedCaptureBonePos = { land_->operator[](( YOLO_POSE_INDEX ) i).x,
					land_->operator[](( YOLO_POSE_INDEX ) i).y,land_->operator[](( YOLO_POSE_INDEX ) i).z };
		capturedata_[ ( YOLO_POSE_INDEX ) i ].captureBonePos = { land_->operator[](( YOLO_POSE_INDEX ) i).x,
					land_->operator[](( YOLO_POSE_INDEX ) i).y,land_->operator[](( YOLO_POSE_INDEX ) i).z }; 
	}
}

void MCB::Capture::Finalize()
{
	m_YOLOPoseEstimation_->End();
	//cv::destroyWindow(windowName.c_str());
}

MCB::CaptureData& MCB::Capture::GetCaptureData(YOLO_POSE_INDEX key)
{
	return capturedata_[ key ];
}

void MCB::Capture::SetYOLOEstimation(YOLOPoseEstimation* yoloEs)
{
	m_YOLOPoseEstimation_ = yoloEs;
}
