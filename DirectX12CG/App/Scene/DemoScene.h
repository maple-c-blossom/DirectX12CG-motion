#pragma once
#include "IScene.h"
#define _USE_MATH_DEFINES
#include "IgnoreWarning.h"
WarningIgnoreBegin
#include <cmath>
WarningIgnoreEnd
#include "CaptureManager.h"
#include "DemoCamera.h"
#include "Player.h"
namespace MCB
{
	class DemoScene :public IScene
	{
		int64_t pad;
	private:
#pragma region 変換行列
		//変換行列
		DemoCamera camera_;

#pragma endregion 変換行列

#pragma region 各種リソース
		//3Dモデル
#pragma region 3Dモデル
		std::unique_ptr<Model> groundModel_;
		std::unique_ptr<Model> skydomeModel_;
		std::unique_ptr<Model> sphereModel_;
		std::unique_ptr<Model> boxModel_;

		std::unique_ptr<AnimationModel> animModel_;

		std::unique_ptr<AnimationModel> anim2Model_;
#pragma endregion 3Dモデル

		//テクスチャ
#pragma region テクスチャ
		TextureCell* debugTextTexture_;
		TextureCell* enter;

#pragma endregion テクスチャ
		bool debugView_;
		bool objInvisibleView_;
		std::array<bool,4> isIk_;
		std::array<bool,4> noMove = { {true,true,true,true} };
		Byte6 pad2;
		//サウンド
#pragma region サウンド
		size_t selectSound_;
		size_t test2Sound_;
		
		int32_t volume_ = 255;

#pragma endregion サウンド

#pragma endregion 各種リソース

#pragma region 3Dオブジェクト
		float animationSpeed;
		int64_t pad3;
		Object3d ground_;
		Object3d Skydorm_;
		std::array<Object3d,4> effectorObjects_;
		std::array<DirectX::XMFLOAT3,4> effectorPos = { DirectX::XMFLOAT3(0,3.45f,0),
			DirectX::XMFLOAT3(-0.6f,3.45f,-0.48f),
			DirectX::XMFLOAT3(0.14f,1.8f,0.05f),DirectX::XMFLOAT3(-0.1f,1.8f,0.0f)};
		Object3d test2Animation_;
		Player play_;
		std::array<DirectX::XMFLOAT3,4> poleVec_ = { DirectX::XMFLOAT3(0.4f,3.44f,-0.05f),
			DirectX::XMFLOAT3(-0.4f,3.44f,-0.05f),
			DirectX::XMFLOAT3(0.23f,2.6f,-0.16f),DirectX::XMFLOAT3(-0.23f,2.6f,-0.16f)
		};
		std::array<Object3d,4> poleVecObjects_;
		std::array<bool,4> PoleVecMove_ = { {false,false,false,false} };
		std::array<bool,4> collIK = { {false,false,false,false} };
		bool gizmoDraw_ = true;
		uint8_t animationNum = 4;
		Byte1 pad5;
		struct IKDataSet
		{
			std::string endJointName = "NULL";
			std::string middleJointName = "NULL";
			std::string rootJointName = "NULL";
			IKDataSet();
			IKDataSet(std::string endJoint);
			IKDataSet(std::string endJoint,std::string middlejoint);
			IKDataSet(std::string endJoint,std::string middleJoint,std::string rootJoint);
		};
		std::array<IKDataSet,4> ikBoneName_ = { std::string("Hand.L"),
														{"mixamorig:RightHand"},{"mixamorig:LeftFoot"},{"mixamorig:RightFoot"} };

		std::array<IKDataSet,4> ikBoneName_2 = { std::string("hand.L"),
											{"Bone.001"},{"Bone.001"},{"Bone.001"} };
		DirectX::XMFLOAT3 scale2 = { 0.1f,0.1f,0.1f };
		int64_t pad6;
		std::array<std::string,5> animationName = { { "Idle", "Run","Jump","Climb","Tpose"} };
		int64_t pad114514;
#pragma endregion 3Dオブジェクト

#pragma region スプライト
		Sprite sprite_;

#pragma endregion スプライト

#pragma region 通常変数
		bool loopFlag_ = true;
		bool chengeModel = false;
		bool startPositionReset_ = true;
		bool debugStop = false;
		bool animePlay = false;

		bool poseInitialize_ = false;
		std::chrono::system_clock::time_point initializetime_;
		std::chrono::system_clock::time_point initializeCount_;

		Byte6 pad7;
		Byte6 pad8;

		CaptureManager cap;
#pragma endregion 通常変数
	public:

		DemoScene(RootParameter* root, Depth* depth,PipeLineManager* pipeline);
		DemoScene(const DemoScene&) = delete;
		DemoScene& operator=(const DemoScene&) = delete;
		~DemoScene();
		void Initialize() override;

		//各初期化系関数群--------------------
		void LoadModel()  override;
		void LoadTexture()  override;
		void LoadSound()  override;
		void Object3DInit()  override;
		void SpriteInit()  override;
		void ParticleInit()  override;
		std::unique_ptr<IScene> GetNextScene() override;
		//---------------
		void MatrixUpdate() override;
		void Update() override;
		void PostEffectDraw() override;
		void Draw() override;
		void SpriteDraw() override;
		void ParticleDraw() override;
		void CheckAllColision()override;
		void ImGuiUpdate()override;
	};
}

