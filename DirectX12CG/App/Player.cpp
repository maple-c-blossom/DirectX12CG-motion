#include "Player.h"
#include "Input.h"
#include "CollisionManager.h"
#include "PlayerQueryCallBack.h"
#include "AnimationModel.h"
#include "Sound.h"
#include "ImGuiManager.h"
#include "MoveBlock.h"
#include "JumpPad.h"
#include "ModelManager.h"
#include "GrabQuery.h"

using namespace std;
float MCB::Player::GetSpeed()
{
	return speedFront_;

}
void MCB::Player::SetRespowPosition(const Vector3D& pos)
{
	respownPosition_ = pos;
}
void MCB::Player::Init()
{
	Object3d::Init();

	for ( auto& itr : moveEffectors )
	{
		itr.Init();
	}

	scale_ = { 0.01f,0.01f,0.01f };
	SetCollider(make_unique<SphereCollider>(Vector3D{ 0,0.5f,0 },0.5f));
	collider_->SetAttribute(ATTRIBUTE_FLENDRY);
	UpdateMatrix();
	collider_->Update();
	//position_ = { 0,0,-50 };
	respownPosition_ = position_;
	rotation_.y = ConvertRadius(180);
	rotation_.x = ConvertRadius(0);
	LightGroup::GetInstance()->SetCShadowActive(0,true);
	rotationQ_.SetRota(Vector3D(0,1,0),ConvertRadius(180));
	currentAnimation_ = "Idle";
}

void MCB::Player::UniqueUpdate()
{
	//slights_->SetSLightPos(0,{ position_.x, position_.y + 1.f, position_.z });
	prevIsClimb = isClimb;

	//slights_->SetSLightIsActive(0,true);
	if ( !isDebug_ )
	{
		if ( !back )
		{
			Move();
		}
	}

	Object3d::UpdateMatrix();
	SphereCollider* sphere = dynamic_cast< SphereCollider* >( collider_ );
	assert(sphere);

	Ray ray;
	ray.StartPosition_ = sphere->centerPosition_;
	ray.StartPosition_.vec_.y_ = sphere->centerPosition_.vec_.y_ - 1.f;
	ray.StartPosition_.vec_.y_ += sphere->GetRaius();
	ray.rayVec_ = { {{0,-1,0,0}} };

	RayCastHit info;
	float distRange = sphere->GetRaius() * 2.0f + 0.05f;
	float distOverRange = distRange * distoffSet;
	bool isGround = CollisionManager::GetInstance()->Raycast(ray,ATTRIBUTE_LANDSHAPE,&info,distRange + distOverRange);
	if ( isGraund_ )
	{
		//const float absDistance = 0.2f;

		if ( isGround )
		{
			isGraund_ = true;
			if ( info.objctPtr_ == climbObj && isClimb )
			{
				isGraund_ = false;
			}
			if ( isGraund_ )
			{
				if ( !isJump && !isClimb )position_.y -= fallV_.vec_.y_;
				if ( info.dist_ <= 1.f - distOverRange )
				{
					position_.y -= ( ( info.dist_ ) - sphere->GetRaius() * 2.0f - 0.05f );

				}
				Object3d::UpdateMatrix();
				if ( info.objctPtr_ )
				{
					if ( info.objctPtr_->nameId_ == "JumpPad" )
					{
						isGraund_ = false;
						JumpPad* ptr = static_cast< JumpPad* >( info.objctPtr_ );
						fallV_ = ptr->jumpVec;
					}
				}
			}
		}
		else
		{
			isGraund_ = false;
			//if(currentAnimation_ != "Jump" )fallV_ = {{{0,0,0,0}}};
		}
	}
	else if ( fallV_.vec_.y_ <= 0.0f )
	{

		if ( isGround )
		{

			isGround = CollisionManager::GetInstance()->Raycast(ray,ATTRIBUTE_LANDSHAPE,&info,distRange + distOverRange + 0.25f);
			isGraund_ = true;
			if ( info.objctPtr_ == climbObj && isClimb )
			{
				isGraund_ = false;
			}
			if ( isGraund_ )
			{
				if ( info.dist_ <= 1.f - distOverRange - 0.25f ) position_.y -= ( ( info.dist_ - distOverRange - 0.05f )
					- sphere->GetRaius() * 2.0f - 0.5f );
				Object3d::UpdateMatrix();
				if ( info.objctPtr_ )
				{
					if ( info.objctPtr_->nameId_ == "JumpPad" )
					{
						JumpPad* ptr = static_cast< JumpPad* >( info.objctPtr_ );
						isGraund_ = false;
						fallV_ = ptr->jumpVec;
						isMoveBlock = false;
					}
					else if ( info.objctPtr_->nameId_ == "MoveBlockUP" || info.objctPtr_->nameId_ == "MoveBlock" )
					{
						if ( !info.objctPtr_->updated )
						{
							info.objctPtr_->UniqueUpdate();
						}
						moveBlock = dynamic_cast< MoveBlock* >( info.objctPtr_ );
						isMoveBlock = true;
					}
					else if ( !isClimb )
					{
						isMoveBlock = false;
					}


				}
			}

		}
		else if ( !isClimb )
		{
			isGraund_ = false;
			isMoveBlock = false;
		}
	}

	CollisionManager::GetInstance()->Raycast(ray,ATTRIBUTE_LANDSHAPE,&info,distRange + distOverRange + 0.25f + 0.09f);
	if ( info.objctPtr_ )
	{
		if ( info.objctPtr_->nameId_ == "MoveBlockUP" || info.objctPtr_->nameId_ == "MoveBlock" )
		{
			if ( !info.objctPtr_->updated )
			{
				info.objctPtr_->UniqueUpdate();
			}
			moveBlock = dynamic_cast< MoveBlock* >( info.objctPtr_ );
			isMoveBlock = true;
		}
		else if ( !isClimb )
		{
			isMoveBlock = false;
		}
	}



	if ( isMoveBlock )
	{

		if ( !moveBlock->updated )
		{
			moveBlock->UniqueUpdate();
		}


		position_.x += moveBlock->totalMoveVec.vec_.x_;
		position_.y += moveBlock->totalMoveVec.vec_.y_;
		position_.z += moveBlock->totalMoveVec.vec_.z_;

	}
	if ( !isMoveBlock )
	{
		moveBlock = nullptr;
	}
	if ( moveBlock )
	{
		moveBlock->isMove = true;
	}


	PlayerQueryCallBack callback(sphere);

	CollisionManager::GetInstance()->QuerySphere(*sphere,&callback,ATTRIBUTE_LANDSHAPE);

	if ( callback.move.V3Len() >= distoffSet && !isClimb )
	{

		position_.x += callback.move.vec_.x_ * 1.4f;
		position_.y += callback.move.vec_.y_ * 1.4f;
		position_.z += callback.move.vec_.z_ * 1.4f;
	}
	if ( callback.obj_ )
	{
		if ( callback.obj_->nameId_ == "checkPoint" )
		{
			respownPosition_ = callback.obj_->position_;
		}
	}

	if ( back )
	{
		backTimer.SafeUpdate();
		position_.x += backVec.vec_.x_;
		position_.y += backVec.vec_.y_;
		position_.z += backVec.vec_.z_;
		if ( backTimer.IsEnd() )
		{
			back = false;
		}
	}
	else if ( callback.block )
	{
		backTimer.Set(10);
		backVec.vec_.z_ = -0.5f;
		back = callback.block;
	}

	if ( isClimb )
	{
		isClimb = false;
		for ( int i = 0; i < 4; i++ )
		{
			handFootSphere[ i ].centerPosition_ = animationModel_->skeleton.GetNode(boneName[ i ])->worldPosition;
			handFootSphere[ i ].radius_ = 0.5f;
			GrabQuery query(&handFootSphere[ i ]);
			CollisionManager::GetInstance()->QuerySphere(handFootSphere[ i ],&query,ATTRIBUTE_LANDSHAPE);
			if ( query.grab )
			{
				Vector3D vec(position_,query.obj_->position_);
				if ( vec.V3Len() >= 0.5f )
				{
					if ( i >= LF )
					{
						if ( query.obj_->position_.y < position_.y )
						{
							moveEffectors[ i ].position_ = query.obj_->position_;
						}
					}
					else
					{
						if ( query.obj_->position_.y >= position_.y )
						{
							moveEffectors[ i ].position_ = query.obj_->position_;

						}
					}
				}
				grabed[ i ] = true;
				isClimb = true;
			}
			else
			{
				grabed[ i ] = false;
			}
		}
	}

	if ( position_.y < outYPosition || isRespown_ )
	{
		position_ = respownPosition_.ConvertXMFloat3();
	}



	slights_->SetCShadowActive(0,true);
	slights_->SetCShadowDistCasterL(0,distanceCasterLight_);
	slights_->SetCShadowFascorAngle(0,factorAngleCos_);
	slights_->SetCShadowAtten(0,atten_);
	slights_->SetCShadowCasterPos(0,{ position_.x, position_.y, position_.z });
}


void MCB::Player::Move()
{
	//SoundManager* sm = SoundManager::GetInstance();
	if ( !input_->IsKeyDown(DIK_S) && !input_->IsKeyDown(DIK_W) &&
		!input_->gamePad_->RTrriger_.x_ && !input_->gamePad_->LTrriger_.x_ )
	{
		speedFront_ = 0;
	}

	if ( input_->IsKeyDown(DIK_W) )
	{
		speedFront_ = maxspeed_;
	}

	if ( input_->IsKeyDown(DIK_S) )
	{
		speedFront_ = -maxspeed_;
	}

	if ( input_->gamePad_->LStick_.y_ )
	{
		float accelerator = maxspeed_;
		accelerator *= input_->gamePad_->LStick_.y_;
		if ( accelerator > 0 ) speedFront_ = accelerator;
		else if ( accelerator < 0 )speedFront_ = accelerator;
	}





	if ( input_->IsKeyDown(DIK_D) )
	{
		speedRight_ = maxspeed_ / 2;
	}

	if ( input_->IsKeyDown(DIK_A) )
	{
		speedRight_ = -maxspeed_ / 2;
	}

	if ( input_->gamePad_->LStick_.x_ )
	{
		float accelerator = maxspeed_;
		accelerator *= input_->gamePad_->LStick_.x_;
		if ( accelerator > 0 ) speedRight_ = accelerator;
		else if ( accelerator < 0 )speedRight_ = accelerator;
	}

	if ( !input_->IsKeyDown(DIK_D) && !input_->IsKeyDown(DIK_A) && !input_->gamePad_->LStick_.x_ )
	{
		speedRight_ = 0;
	}

	if ( abs(speedFront_) > abs(speedRight_) )
	{
		if ( speedFront_ != 0 )
		{
			if ( speedFront_ > 0 )currentAnimation_ = "Run";
			if ( speedFront_ < 0 )currentAnimation_ = "Run";

		}
	}
	else if ( speedRight_ != 0 )
	{
		if ( speedRight_ > 0 )currentAnimation_ = "Run";
		if ( speedRight_ < 0 )currentAnimation_ = "Run";
	}
	else
	{
		currentAnimation_ = "Idle";

	}
	if ( currentAnimation_ == "Idle" )
	{
		animationSpeed_ = 0.005f;
	}
	else
	{
		animationSpeed_ = max(abs(speedFront_) / 10,abs(speedRight_) / 10);
	}
	if ( !isGraund_ )
	{
		jumpokTimer.Update();
		if ( jumpokTimer.IsEnd() )
		{
			const float fallAcc = -0.025f;
			const float VYMin = -0.75f;
			fallV_.vec_.y_ = max(fallV_.vec_.y_ + fallAcc,VYMin);

			animationPositionRock = true;
			isJump = false;
			currentAnimation_ = "Jump";
			animationSpeed_ = 0.015f;
			if ( animeTime_ >= 1.160f )
			{
				animationSpeed_ = 0;
			}
		}
	}
	else
	{
		jumpokTimer.Set(5);
	}

	if ( ( Input::GetInstance()->IsKeyDown(DIK_SPACE) || input_->gamePad_->IsButtonDown(GAMEPAD_A) ) &&
		!isJump && !isClimb && !jumpokTimer.IsEnd() )
	{
		isJump = true;
	}
	else if ( isGraund_ )
	{
		if ( currentAnimation_ != "Jump" )
		{
			fallV_.vec_.y_ = -0.075f;
		}
		animationPositionRock = true;
	}


	if ( isJump )
	{
		currentAnimation_ = "Jump";
		animationPositionRock = true;
		//speedFront_ = 0;
		animeTime_ = 0.09f;
		animationSpeed_ = 0.015f;
		if ( animeTime_ + animationSpeed_ >= 0.09f )
		{
			const float jumpVYFist = 0.4f;
			fallV_ = { {{0,jumpVYFist,0,0}} };
		}
	}

	/*if ( fallV_.vec_.y_ < 0 && animeTime_ < 0.979f && currentAnimation_ == "Jump" && !isClimb)
	{
		animeTime_ = 0.979f;
	}*/

	wallCheckRay.StartPosition_ = position_;
	wallCheckRay.rayVec_ = nowFrontVec_;

	upperCheckRay.StartPosition_ = position_;
	upperCheckRay.StartPosition_.vec_.y_ += 1.5f;

	upperCheckRay.rayVec_ = nowFrontVec_;
	prevWallHit_ = wallHit_;
	RayCastHit info;
	wallHit_ = CollisionManager::GetInstance()->Raycast(wallCheckRay,ATTRIBUTE_LANDSHAPE,&info,0.35f);
	//bool upperHit = CollisionManager::GetInstance()->Raycast(upperCheckRay,ATTRIBUTE_WALL,nullptr,0.15f);

	if ( info.objctPtr_ )
	{
		effectorPos.vec_.y_ = info.objctPtr_->position_.y + info.objctPtr_->scale_.y;
		effectorPos.vec_.z_ = info.objctPtr_->position_.z - info.objctPtr_->scale_.z;

		moveEffectors[ LH ].position_ = {
			position_.x - 0.15f,position_.y,effectorPos.vec_.z_
		};

		moveEffectors[ RH ].position_ = {
			position_.x + 0.15f,position_.y,effectorPos.vec_.z_
		};

		if ( info.objctPtr_->nameId_ == "Rblock" )
		{
			wallHit_ = false;
		}
	}

	if ( wallHit_ && !isClimb && ( Input::GetInstance()->IsKeyDown(DIK_SPACE) || input_->gamePad_->IsButtonDown(GAMEPAD_A) ) )
	{
		if ( info.objctPtr_ )
		{
			if ( info.objctPtr_->nameId_ == "MoveBlock" )
			{
				moveBlock = static_cast< MoveBlock* >( info.objctPtr_ );
				isMoveBlock = true;
			}
		}

		climbObj = info.objctPtr_;
		isJump = false;
		fallV_.vec_.y_ = 0;
		position_.z = effectorPos.vec_.z_ - 0.025f;
		//  開始位置を保持
		//climbOldPos = Vector3D(position_.x,effectorPos.vec_.y_ - 2.63f,effectorPos.vec_.z_ - 0.18f);
		climbOldPos = Vector3D(position_.x,position_.y,position_.z - 0.5f);
		position_.x = climbOldPos.vec_.x_;
		position_.y = climbOldPos.vec_.y_;
		position_.z = climbOldPos.vec_.z_;
		//  終了位置を算出
		climbPos = Vector3D(position_.x,effectorPos.vec_.y_ + 0.75f,effectorPos.vec_.z_ + 0.15f);
		//  掴みを解除
		isGrab = false;
		//  よじ登りを実行
		isClimb = true;
		isClimbUp = false;
		upOk = false;
		moveBlockTotal = { 0,0,0 };

		animeTime_ = 0.f;
		wallUPTimer.Set(uptime);

		poleVec[ LH ] = Vector3D(0.1134f,-0.4093f,-0.4241f);
		poleVec[ RH ] = Vector3D(0.1134f,-0.4093f,-0.4241f);
		poleVec[ LF ] = Vector3D(position_.x - 1.15f,position_.y - 1.5f,position_.z + 2.f);
		poleVec[ RF ] = Vector3D(position_.x + 1.15f,position_.y - 1.5f,position_.z + 2.f);
		animationModel_->skeleton.GetNode("mixamorig:LeftHand")->lineView = true;
		animationModel_->skeleton.GetNode("mixamorig:RightHand")->lineView = true;
	}
	//else if(wallHit_)
	//{
	//	if (/* !prevWallHit_ ||*/isGraund_) 
	//	{
	//		wallTimer.Set(10);
	//	}
	//	wallTimer.Update();
	//	if ( !wallTimer.IsEnd() )
	//	{
	//		//fallV_.vec_.y_ = 0;
	//		//fallV_.vec_.y_ = speedFront_;
	//	}
	//}

	if ( currentAnimation_ == "Run" || currentAnimation_ == "Idle" || currentAnimation_ == "Tpose" )
	{
		animationLoop_ = true;
	}
	else
	{
		animationLoop_ = false;
	}
	if ( input_->gamePad_->IsButtonDown(GAMEPAD_B) && isClimb )
	{
		isClimb = false;
		for ( int i = 0; i < 4; i++ )
		{
			animationModel_->skeleton.TwoBoneIKOff(boneName[ i ]);
		}
	}
	if ( isClimb )
	{

		isJump = false;
		animationPositionRock = isRootStop;
		currentAnimation_ = "Climb";
		//currentAnimation_ = "Tpose";
		if ( isIkClimb )
		{


			for ( int i = 0; i < 4; i++ )
			{
				animationModel_->skeleton.SetTwoBoneIK(*this,
					{ moveEffectors[ i ].position_.x,moveEffectors[ i ].position_.y,moveEffectors[ i ].position_.z },
					poleVec[ i ],
					boneName[ i ],"NULL","NULL",i <= RH);
				//animationModel_->skeleton.GetNode(boneName[ i ])->ikData.computeConstraintVec = true;
			}

		}
		//y0.7,z54.8;

		if ( !isClimbUp && prevIsClimb && ( input_->IsKeyTrigger(DIK_SPACE) || input_->gamePad_->IsButtonTrigger(GAMEPAD_A) ) )
		{
			isClimbUp = true;
		}

		if ( !isClimbUp || !upOk ) animationSpeed_ = 0;
		else animationSpeed_ = 0.015f;
		if ( isClimbUp )
		{
			if ( upOk )
			{

				if ( !animationPositionRock )
				{
					position_.x = climbOldPos.vec_.x_;
					position_.y = climbOldPos.vec_.y_;
					position_.z = climbOldPos.vec_.z_;

				}
				wallUPTimer.Update(1);



				if ( animationPositionRock )
				{

					animationSpeed_ = 1.377f / static_cast< float >( uptime );
					float time = animeTime_ + animationSpeed_;

					// 前後は後半にかけて早く移動する//前後も始まりずれてる。速度も少しずれている
					if ( climbFrontMove )
					{
						if ( time <= 0.45f )
						{

							position_.z = static_cast< float >( Lerp(climbOldPos.vec_.z_,climbOldPos.vec_.z_ - 0.45f,
								0.45f,time) );
						}
						else if ( time >= 0.45f && time <= 1.145f )
						{
							float prevz = position_.z;
							position_.z = static_cast< float >( Lerp(climbOldPos.vec_.z_ - 0.45f,climbPos.vec_.z_ + 0.7f,
								1.145f - 0.45f,time - 0.45f) );

							climbMove.vec_.z_ = position_.z - prevz;
							animationModel_->skeleton.SetTwoBoneIK(*this,
								{ position_.x - 0.15f,effectorPos.vec_.y_,effectorPos.vec_.z_ + climbMove.vec_.z_ },poleVec[ LH ],
								"mixamorig:LeftHand","NULL","NULL",true);

							animationModel_->skeleton.SetTwoBoneIK(*this,
								{ position_.x + 0.15f ,effectorPos.vec_.y_,effectorPos.vec_.z_ + climbMove.vec_.z_ },
								poleVec[ RH ],"mixamorig:RightHand","NULL","NULL",true);

							if ( time >= 0.5f )
							{
								animationModel_->skeleton.SetTwoBoneIK(*this,
									{ position_.x - 0.15f,effectorPos.vec_.y_,effectorPos.vec_.z_ },poleVec[ LH ],
									"mixamorig:LeftFoot","NULL","NULL",true);
								animationModel_->skeleton.SetTwoBoneIK(*this,
									{ position_.x + 0.15f ,effectorPos.vec_.y_,effectorPos.vec_.z_ },poleVec[ RH ],
								"mixamorig:RightFoot","NULL","NULL",true);
							}

						}
					}


					if ( climbUpMove )
					{
						if ( time <= 0.299f )
						{
							position_.y = static_cast< float >( Lerp(climbOldPos.vec_.y_,climbPos.vec_.y_ - 0.486f,
								0.299f,time) );
						}

						else if ( time >= 0.299f && time <= 0.4f )
						{

							position_.y = static_cast< float >( Lerp(climbPos.vec_.y_ - 0.486f,climbPos.vec_.y_,
								0.4f - 0.299f,time - 0.299f) );
						}
						if ( time + animationSpeed_ > 1.145f )
						{
							//animationModel_->skeleton.TwoBoneIKOff("mixamorig:LeftHand");
							//animationModel_->skeleton.TwoBoneIKOff("mixamorig:RightHand");
						}

					}

				}
			}
			else
			{
				DirectX::XMFLOAT3 prevPos = position_;
				if ( input_->gamePad_->IsButtonDown(GAMEPAD_LB) && grabed[ LH ] )
				{
					Node* node = animationModel_->skeleton.GetNode(boneName[ LH ]);
					position_.x = InQuad(oldMovePos.vec_.x_,node->worldPosition.vec_.x_,
						moveTimer.GetEndTime(),moveTimer.NowTime());
					position_.y = InQuad(oldMovePos.vec_.y_,node->worldPosition.vec_.y_,
						moveTimer.GetEndTime(),moveTimer.NowTime());
				}
				else if ( grabed[ RH ] )
				{
					Node* node = animationModel_->skeleton.GetNode(boneName[ RH ]);
					position_.x = InQuad(oldMovePos.vec_.x_,node->worldPosition.vec_.x_,
						moveTimer.GetEndTime(),moveTimer.NowTime());
					position_.y = InQuad(oldMovePos.vec_.y_,node->worldPosition.vec_.y_,
						moveTimer.GetEndTime(),moveTimer.NowTime());
				}
				DirectX::XMFLOAT3 moveLevel = { position_.x - prevPos.x, position_.y - prevPos.y, position_.z - prevPos.z };
				for ( int i = 0; i < 4; i++ )
				{
					moveEffectors[ i ].position_.x += moveLevel.x;
					moveEffectors[ i ].position_.y += moveLevel.y;
				}
				moveTimer.SafeUpdate();
				if ( moveTimer.IsEnd() )
				{

					isClimbUp = false;
				}
				climbOldPos = Vector3D(position_.x,position_.y,position_.z - 0.5f);




			}
		}
		else
		{
			if ( developmentMode == false )
			{
				upOk = true;
				animationModel_->skeleton.SetTwoBoneIK(*this,
					{ position_.x - 0.15f ,effectorPos.vec_.y_,effectorPos.vec_.z_ },
						poleVec[ LH ],
						"mixamorig:LeftHand","NULL","NULL",true);
				animationModel_->skeleton.SetTwoBoneIK(*this,
					{
					position_.x + 0.15f ,effectorPos.vec_.y_,effectorPos.vec_.z_ },
					poleVec[ RH ],
					"mixamorig:RightHand","NULL","NULL",true);
			}

			if ( !upOk )
			{


				if ( input_->gamePad_->LStick_.x_ )
				{
					float accelerator = maxspeed_;
					accelerator *= input_->gamePad_->LStick_.x_;
					if ( input_->gamePad_->IsButtonDown(GAMEPAD_LB) )
					{
						moveEffectors[ LF ].position_.x += accelerator;
					}
					else
					{
						moveEffectors[ LH ].position_.x += accelerator;
					}
					//moveEffectors[RH].position_.y += accelerator;
				}

				if ( input_->gamePad_->LStick_.y_ )
				{
					float accelerator = maxspeed_;
					accelerator *= input_->gamePad_->LStick_.y_;
					if ( abs(effectorPos.vec_.y_ - position_.y) >= 0.25f || accelerator < 0 )
					{
						if ( input_->gamePad_->IsButtonDown(GAMEPAD_LB) )
						{
							moveEffectors[ LF ].position_.y += accelerator;
						}
						else
						{
							moveEffectors[ LH ].position_.y += accelerator;
						}
					}
				}


				if ( input_->gamePad_->RStick_.x_ && !input_->gamePad_->IsButtonDown(GAMEPAD_RB) )
				{
					float accelerator = maxspeed_;
					accelerator *= input_->gamePad_->RStick_.x_;
					if ( input_->gamePad_->IsButtonDown(GAMEPAD_LB) )
					{
						moveEffectors[ RF ].position_.x += accelerator;
					}
					else
					{
						moveEffectors[ RH ].position_.x += accelerator;
					}
					//moveEffectors[ RH ].position_.y += accelerator;
				}

				if ( input_->gamePad_->RStick_.y_ )
				{
					float accelerator = maxspeed_;
					accelerator *= input_->gamePad_->RStick_.y_;
					if ( abs(effectorPos.vec_.y_ - position_.y) >= 0.25f || accelerator < 0 )
					{
						if ( input_->gamePad_->IsButtonDown(GAMEPAD_LB) && !input_->gamePad_->IsButtonDown(GAMEPAD_RB) )
						{
							moveEffectors[ RF ].position_.y += accelerator;
						}
						else
						{
							moveEffectors[ RH ].position_.y += accelerator;
						}
					}
				}

				if ( abs(effectorPos.vec_.y_ - position_.y) <= 0.25 )
				{
					upOk = true;
					animationModel_->skeleton.SetTwoBoneIK(*this,
						{ effectorPos.vec_.x_ - 0.15f ,effectorPos.vec_.y_,effectorPos.vec_.z_ },
						poleVec[ LH ],
						"mixamorig:LeftHand","NULL","NULL",true);
					animationModel_->skeleton.SetTwoBoneIK(*this,
						{
						effectorPos.vec_.x_ + 0.15f ,effectorPos.vec_.y_,effectorPos.vec_.z_ },
						poleVec[ RH ],
						"mixamorig:RightHand","NULL","NULL",true);
				}

				//position_.z += nowFrontVec_.vec_.z_ * speedFront_;
				rightVec_ = rightVec_.GetRightVec(nowFrontVec_);
				for ( int i = 0; i < 4; i++ )
				{
					moveEffectors[ i ].position_.x += rightVec_.vec_.x_ * effectorMove[ i ];
				}

				float areamax = climbObj->position_.x + climbObj->scale_.x;
				float areamin = climbObj->position_.x - climbObj->scale_.x;

				float moveMaxLen = 0.75f;
				Vector3D moveVec(Float3{ position_.x,position_.y,0 },
					{ moveEffectors[ LH ].position_.x,moveEffectors[ LH ].position_.y,0 });

				if ( moveVec.V3Len() > moveMaxLen )
				{
					moveVec.V3Norm();
					moveVec = moveVec * 1.5f;
					moveEffectors[ LH ].position_.x = moveVec.vec_.x_ + position_.x;
					moveEffectors[ LH ].position_.y = moveVec.vec_.y_ + position_.y;
				}

				moveVec = Vector3D(Float3{ position_.x,position_.y,0 },
					{ moveEffectors[ RH ].position_.x,moveEffectors[ RH ].position_.y,0 });

				if ( moveVec.V3Len() > moveMaxLen )
				{
					moveVec.V3Norm();
					moveVec = moveVec * 1.5f;
					moveEffectors[ RH ].position_.x = moveVec.vec_.x_ + position_.x;
					moveEffectors[ RH ].position_.y = moveVec.vec_.y_ + position_.y;
				}



				if ( isGraund_ )
				{
					isClimb = false;
					climbObj = nullptr;
					animationModel_->skeleton.TwoBoneIKOff("mixamorig:LeftHand");
					animationModel_->skeleton.TwoBoneIKOff("mixamorig:RightHand");
					animationModel_->skeleton.TwoBoneIKOff("mixamorig:LeftFoot");
					animationModel_->skeleton.TwoBoneIKOff("mixamorig:RightFoot");
					//position_.x = climbPos.vec_.x_;
					//position_.y = climbPos.vec_.y_;
					//position_.z = climbPos.vec_.z_ + 0.7f;//最後に目的の場所に最終調整
					fallV_.vec_.y_ = 0.0f;
					isJump = false;
				}

				//position_.z += rightVec_.vec_.z_ * speedRight_;
				oldMovePos = position_;
				moveTimer.Set(30);
			}

		}

		float time = animeTime_ + animationSpeed_;
		//  座標を更新
		fallV_.vec_.y_ = 0.0f;
		if ( wallUPTimer.IsEnd() || time >= 1.377f )
		{
			isClimb = false;
			animationModel_->skeleton.TwoBoneIKOff("mixamorig:LeftHand");
			animationModel_->skeleton.TwoBoneIKOff("mixamorig:RightHand");
			animationModel_->skeleton.TwoBoneIKOff("mixamorig:LeftFoot");
			animationModel_->skeleton.TwoBoneIKOff("mixamorig:RightFoot");
			//position_.x = climbPos.vec_.x_;
			//position_.y = climbPos.vec_.y_;
			//position_.z = climbPos.vec_.z_ + 0.7f;//最後に目的の場所に最終調整
			fallV_.vec_.y_ = 0.0f;
			isJump = false;
		}
	}


	if ( !isClimb )
	{
		if ( !( speedFront_ > 0 && wallHit_ ) )
		{
			position_.x += nowFrontVec_.vec_.x_ * speedFront_;
			position_.z += nowFrontVec_.vec_.z_ * speedFront_;
			rightVec_ = rightVec_.GetRightVec(nowFrontVec_);
			position_.x += rightVec_.vec_.x_ * speedRight_;
			position_.z += rightVec_.vec_.z_ * speedRight_;

			if ( speedFront_ != 0 || speedRight_ != 0 )
			{
				moving = true;
			}
			else
			{
				moving = false;
			}
		}
		else
		{
			moving = false;
		}
	}
	else if ( isClimb )
	{
		moving = true;
		directionVec = Vector3D(0,0,1);
	}
	if ( moving )
	{
		if ( isClimb ) directionVec = Vector3D(0,0,1);
		else
		{
			directionVec = Vector3D(speedRight_,0,speedFront_);
		}
		if ( directionVec.V3Len() == 0 )
		{
			directionVec = Vector3D(0,0,1);
		}
	}
	directionVec.V3Norm();

	if ( directionVec.vec_.x_ == 0 )
	{
		if ( directionVec.vec_.z_ > 0 )
		{
			rotationQ_.SetRota(Vector3D(0,1,0),ConvertRadius(180));
		}
		else if ( directionVec.vec_.z_ < 0 )
		{
			rotationQ_.SetRota(Vector3D(0,1,0),ConvertRadius(0));
		}
	}
	else
	{
		rotationQ_ = rotationQ_.DirToDir(Vector3D(0,0,-1),directionVec);
		camera_->targetObjctVec_.vec_.y_ = 0;
		Quaternion q = q.DirToDir(Vector3D(0,0,1),camera_->targetObjctVec_.GetV3Norm());
		rotationQ_ = q.GetDirectProduct(rotationQ_,q);
	}
	//rotationQ_ = rotationQ_.GetReciprocal(rotationQ_);
	if ( !isClimb )
	{
		position_.x += fallV_.vec_.x_;
		position_.y += fallV_.vec_.y_;
		position_.z += fallV_.vec_.z_;

	}

}

void MCB::Player::Debug()
{

	ImGui::InputFloat("Dist",&distanceCasterLight_);
	slights_->SetCShadowDistCasterL(0,distanceCasterLight_);
	float temp[ 2 ] = { factorAngleCos_.x_,factorAngleCos_.y_ };
	ImGui::InputFloat2("angle",temp);
	factorAngleCos_.x_ = temp[ 0 ]; factorAngleCos_.y_ = temp[ 1 ];
	slights_->SetCShadowFascorAngle(0,factorAngleCos_);

	float temp2[ 3 ] = { atten_.x_,atten_.y_ ,atten_.z_ };
	ImGui::InputFloat2("atten",temp2);
	atten_.x_ = temp2[ 0 ]; atten_.y_ = temp2[ 1 ]; atten_.z_ = temp2[ 2 ];
	slights_->SetCShadowAtten(0,atten_);

	if ( ImGui::TreeNode("プレイヤーデバッグ") )
	{
		ImGui::Checkbox("Debug",&isDebug_);
		ImGui::Checkbox("DevelopMode",&developmentMode);
		ImGui::Checkbox("climbUpMove",&climbUpMove);
		ImGui::Checkbox("climbFrontMove",&climbFrontMove);
		ImGui::Checkbox("isRootStop",&isRootStop);
		ImGui::Checkbox("ObjectSideView",&cameraViewFromSide_);
		ImGui::Checkbox("isIk",&isIkClimb);
		ImGui::Text("EffectorとPoleVectorまでの線を描画左");
		ImGui::Checkbox("LineDraw",&animationModel_->skeleton.GetNode("mixamorig:LeftHand")->lineView);
		ImGui::Text("EffectorとPoleVectorまでの線を描画右");
		ImGui::Checkbox("LineDraw",&animationModel_->skeleton.GetNode("mixamorig:RightHand")->lineView);
		currentAnimation_ = animationName[ animationNum ];

		string rootName = animationModel_->animationManager.GetAnimation(currentAnimation_)->animationRootNode;
		Node* root = animationModel_->skeleton.GetNode(rootName);
		Vector3D nodePos;
		if ( root ) nodePos = root->worldPosition;
		ImGui::Text("Positin:%f,%f,%f",position_.x,position_.y,position_.z);
		ImGui::Text("EffectorPositin:%f,%f,%f",effectorPos.vec_.x_,effectorPos.vec_.y_,effectorPos.vec_.z_);
		ImGui::Text("ClimbStartPositin:%f,%f,%f",climbOldPos.vec_.x_,climbOldPos.vec_.y_,climbOldPos.vec_.z_);
		ImGui::Text("animationRootPos:%f,%f,%f",nodePos.vec_.x_,nodePos.vec_.y_,nodePos.vec_.z_);
		ImGui::Text("fallV:%f",fallV_.vec_.y_);
		//position_.y = 2.88f;
		//position_.z = 55.96f;
		//0.73f;
		//local y2.48f,z0.13
		float animTime = animeTime_;
		ImGui::SliderFloat("AnimTime",&animTime,0.f,3.8f);
		animeTime_ = animTime;

		if ( ImGui::TreeNode("ComputePoleVec") )
		{
			for ( uint8_t i = 0; i < 4; i++ )
			{
				ImGui::Text(" ");
				ImGui::Text("ONの時、自動でPoleVecの位置を再計算する");
				string temp = boneName[ i ] + ":ComputePoleVec";
				ImGui::Checkbox(temp.c_str(),&animationModel_->skeleton.GetNode(boneName[ i ])->ikData.computeConstraintVec);
			}
			ImGui::TreePop();
		}

		if ( isDebug_ )
		{
			ImguiManager::GuizmoDraw(this,ImGuizmo::OPERATION::TRANSLATE,ImGuizmo::LOCAL);
			if ( ImGui::BeginCombo("Animation",animationName[ animationNum ].c_str()) )
			{
				for ( uint8_t i = 0; i < animationName.size(); i++ )
				{
					if ( ImGui::Selectable(animationName[ i ].c_str(),i == animationNum) )
					{
						animationNum = i;

					}
				}
				ImGui::EndCombo();
			}
			ImGui::InputFloat("AnimationSpeed",&animationPlaySpeed);
			if ( ImGui::Button("アニメーション再生") )
			{
				animePlay = !animePlay;
				animeTime_ = 0.f;
			}

			if ( animePlay )
			{
				animationSpeed_ = animationPlaySpeed;
			}
			else
			{
				animationSpeed_ = 0;
			}
			rotation_.y = ConvertRadius(180);
			animationPositionRock = isRootStop;
		}

		ImGui::TreePop();
	}
}

void MCB::Player::OnCollision(const CollisionInfomation& info)
{
	if ( info.object3d_->nameId_ == "checkPoint" )
	{
		respownPosition_ = info.object3d_->position_;
	}

}

void MCB::Player::AnimationUpdate(bool isBillBord)
{
	if ( animationModel_ == nullptr )return;
	UpdateMatrix(rotationQ_,isBillBord);

	constMapTranceform_->world = matWorld_.matWorld_;
	constMapTranceform_->cameraMat = camera_->GetView()->mat_ /** animationModel->nodes.begin()->get()->globalTransform*/;
	constMapTranceform_->viewproj = camera_->GetProjection()->mat_;
	constMapTranceform_->cameraPos.x_ = camera_->GetView()->eye_.x;
	constMapTranceform_->cameraPos.y_ = camera_->GetView()->eye_.y;
	constMapTranceform_->cameraPos.z_ = camera_->GetView()->eye_.z;
	constMapTranceform_->shaderNum = shaderNum_;
	constMapTranceform_->color = color_;
	if ( collider_ )collider_->Update();
	animeTime_ += animationSpeed_;

	animationModel_->AnimationUpdate(animeTime_,currentAnimation_,this,animationLoop_,animationPositionRock);
	moveEffectors[ LH ].camera_ = moveEffectors[ RH ].camera_ = camera_;
	moveEffectors[ LH ].Update(false);
	moveEffectors[ RH ].Update(false);


}

void MCB::Player::Draw()
{
	//moveEffectorLeftHand.Draw();
	//moveEffectorRightHand.Draw();
}

