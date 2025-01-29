#pragma once
#include "Vector3.h"
#include <stdint.h>

namespace MCB
{
	class Quaternion
	{
	public:
		float x = 0.f;
		float y = 0.f;
		float z = 0.f;
		float w = 1.f;
		Quaternion(const Vector3& vec, float angle);
		Quaternion();
		Quaternion(float x, float y, float z, float w);

		//�^����ꂽ�x�N�g������̉�]��\���N�H�[�^�j�I���𐶐�
		void SetRota(Vector3 vec, float angle);
		//����
		Quaternion GetConjugated(Quaternion q);
		//�t���i���w�p����
		Quaternion GetReciprocal(Quaternion q);
		//����
		double GetNorm();
		//q*p�̒���
		Quaternion GetDirectProduct(const Quaternion& q, const Quaternion& p);
		//����
		float Dot(const Quaternion& a, const Quaternion& b);
		//��̃N�H�[�^�j�I���̊p�x��
		float GetAngle(const Quaternion& a, const Quaternion& b, float& dot, bool& nan);
		float GetAngle(const Quaternion& a, const Quaternion& b);
		//���ʐ��`���
		Quaternion Slerp(Quaternion start, const Quaternion& end, int32_t time, int32_t maxTime);
		//���ʐ��`���
		Quaternion Slerp(Quaternion start, Quaternion end, float time);
		//�^����ꂽ�x�N�g��������̉�]��PositionVec�ɗ^����
		Vector3 SetRotationVector(const Vector3& rotationAxisVec, Vector3 PositionVec, float angle);
		//�^����ꂽ�N�H�[�^�j�I���̉�]��PositionVec�ɗ^����
		Vector3 SetRotationVector(const Quaternion& rotationQuaternion, Vector3 PositionVec);


		Vector3 SetRotationVector(const Quaternion& rotationQuaternion, const Quaternion& PositionVec);

		Quaternion DirToDir(Vector3 u, Vector3 v);
		//���K������
		void Normalize();
		//���K�������l��Ԃ�
		static Quaternion Normalize(Quaternion q);
		

		void SinCos(float* returnSin, float* returnCos, float theta);
		//�I�C���[�p����N�H�[�^�j�I���𐶐�
		Quaternion SetToRorateObjectToInternal(const Vector3& eulerAngle);
		//�N�H�[�^�j�I���̉�]�����Z�o
		Vector3 GetRotationAxis(const Quaternion& q);
		//�N�H�[�^�j�I���̉�]�����Z�o(AxisVec������������)
		void GetRotationAxis(const Quaternion& q, Vector3& AxisVec);
		//�P�ʃN�H�[�^�j�I��
		Quaternion Identity();

		//�N�H�[�^�j�I���̊p�x
		float GetAngle(const Quaternion& q);

		float SafeAcos(float a);

		bool operator== (const Quaternion& q);
		Quaternion operator-();
		Quaternion operator*(float k);
		Quaternion operator+(Quaternion q);
	};
	Quaternion operator*(float k, Quaternion q);
	Quaternion SetRota(const Vector3& vec, float angle);
}

