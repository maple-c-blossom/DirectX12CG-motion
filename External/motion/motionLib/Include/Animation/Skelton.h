#pragma once
#include <memory>
#include <unordered_map>

#include "Bone.h"
#include "Capture.h"


namespace MCB
{
	class Skelton
	{
	private:
		std::vector<std::unique_ptr<Bone>> bones_;
		
		Bone* rootBone_;

		std::unique_ptr<Capture> capture;

	public:
		//�J�����̋N�������s��
		void InitializeCapture();

		void AddBone(std::unique_ptr<Bone> bone);

		Bone* GetBone(std::string name);

		Quaternion GetBoneRotation(std::string name);

		//�L���v�`���[�f�[�^�̓ǂݍ���
		void UpdateCaptureData();

		//���f���̏����p���̐ݒ�i�v���C���[�Ƀ��f���̑f�̃|�[�Y���܂˂Ă��炤�j
		void CaptureBasePoseInitialize();

		void SetRootBone(Bone* bone);

		void CaptureBoneAccept();

		/// <summary>
		/// ���f���̉�]�̌v�Z
		/// </summary>
		/// <param name="rootBoneName">�L���v�`�������������{��Key(��:YOLO_POSE_INDEX::SHOULDER_L)</param>
		/// <param name="boneCount">�ǂ̂��炢��[�̃{�[���܂ł�邩�B�r�Ȃ�2�ŗǂ�</param>
		void CaptureBoneUpdate(YOLO_POSE_INDEX rootBoneName, uint32_t boneCount = 2);
	};
}
