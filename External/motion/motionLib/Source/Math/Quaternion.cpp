#include "Quaternion.h"

#include <cmath>
#define PI 3.14159265358979323846264338327950288f
#define FLT_EPSILON      1.192092896e-07F
using namespace MCB;


MCB::Quaternion::Quaternion(const Vector3& vec, float angle)
{
	SetRota(vec, angle);
}

void MCB::Quaternion::SetRota(Vector3 vec, float angle)
{
	vec.V3Norm();
	float s = sinf(angle / 2);
	x = s * vec.x;
	y = s * vec.y;
	z = s * vec.z;
	w = cosf(angle / 2);
	Normalize();
}

MCB::Quaternion::Quaternion()
{
	Identity();
}

MCB::Quaternion::Quaternion(float x, float y, float z, float w)
{
	x = x;
	y = y;
	z = z;
	w = w;
}

Quaternion MCB::Quaternion::GetConjugated(Quaternion q)
{
	q.x *= -1;
	q.y *= -1;
	q.z *= -1;
	return q;
}

Quaternion MCB::Quaternion::GetReciprocal(Quaternion q)
{
	Quaternion tempQ = GetConjugated(q);
	double norm = q.GetNorm();
	norm *= norm;
	tempQ = { tempQ.x / (float)norm, tempQ.y / (float)norm, tempQ.z / (float)norm, tempQ.w / (float)norm };
	return tempQ;
}

double MCB::Quaternion::GetNorm()
{
	return sqrt(x * x + y * y + z * z + w * w);
}

Quaternion MCB::Quaternion::GetDirectProduct(const Quaternion& q, const Quaternion& p)
{
	Quaternion ans;
	ans.x = (q.x * p.w) + (q.y * p.z) - (q.z * p.y) + (q.w * p.x);
	ans.y = (-q.x * p.z) + (q.y * p.w) + (q.z * p.x) + (q.w * p.y);
	ans.z = (q.x * p.y) - (q.y * p.x) + (q.z * p.w) + (q.w * p.z);
	ans.w = (-q.x * p.x) - (q.y * p.y) - (q.z * p.z) + (q.w * p.w);
	ans.Normalize();
	return ans;
}

float MCB::Quaternion::Dot(const Quaternion& a, const Quaternion& b)
{

	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

float MCB::Quaternion::GetAngle(const Quaternion& a, const Quaternion& b, float& dot, bool& nan)
{
	dot = Dot(a, b);
	if (dot < 0)
	{
		//dot *= -1;
		nan = true;
	}
	return acosf(dot);
}

float MCB::Quaternion::GetAngle(const Quaternion& a, const Quaternion& b)
{
	float dot = Dot(a, b);
	if (dot < 0)
	{
		dot *= -1;
	}
	return acosf(dot);
}

Vector3 MCB::Quaternion::SetRotationVector(const Vector3& rotationAxisVec, Vector3 PositionVec, float angle)
{
	PositionVec.V3Norm();
	Quaternion position{};
	position.x = PositionVec.x;
	position.y = PositionVec.y;
	position.z = PositionVec.z;
	position.w = 0;
	Quaternion RotationAngle;
	RotationAngle.SetRota(rotationAxisVec, angle);

	position = GetDirectProduct(RotationAngle, position);
	position = GetDirectProduct(position, GetReciprocal(RotationAngle));

	position.Normalize();
	return Vector3(position.x, position.y, position.z);
}

Vector3 MCB::Quaternion::SetRotationVector(const Quaternion& rotationQuaternion, Vector3 PositionVec)
{
	PositionVec.V3Norm();
	Quaternion position{};
	position.x = PositionVec.x;
	position.y = PositionVec.y;
	position.z = PositionVec.z;
	position.w = 0;

	position = GetDirectProduct(rotationQuaternion, position);
	position = GetDirectProduct(position, GetReciprocal(rotationQuaternion));

	position.Normalize();
	return Vector3(position.x, position.y, position.z);
}

Vector3 MCB::Quaternion::SetRotationVector(const Quaternion& rotationQuaternion, const Quaternion& PositionVec)
{

	Quaternion position{};

	position = GetDirectProduct(rotationQuaternion, PositionVec);
	position = GetDirectProduct(PositionVec, GetReciprocal(rotationQuaternion));

	position.Normalize();
	return Vector3(position.x, position.y, position.z);
}

Quaternion MCB::Quaternion::DirToDir(Vector3 u, Vector3 v)
{
	u.V3Norm();
	v.V3Norm();
	float angle = u.GetV3Dot(v);
	Vector3 axis = u.GetV3Cross(v);
	axis.V3Norm();
	float theta = acosf(angle);
	return Quaternion(axis, theta);
}


void MCB::Quaternion::Normalize()
{
	float mag = (float)sqrt(x * x + y * y + z * z + w * w);

	if (mag > 0.0f)
	{
		float oneOverMag = 1.0f / mag;

		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
		w *= oneOverMag;
	}


}


Quaternion MCB::Quaternion::Normalize(Quaternion q)
{
	q.Normalize();
	return q;
}

void MCB::Quaternion::SinCos(float* returnSin, float* returnCos, float theta)
{
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}

Quaternion MCB::Quaternion::SetToRorateObjectToInternal(const Vector3& eulerAngle)
{
	Quaternion ans;
	float sp, sb, sh;
	float cp, cb, ch;

	SinCos(&sp, &cp, eulerAngle.x * 0.5f);
	SinCos(&sb, &cb, eulerAngle.z * 0.5f);
	SinCos(&sh, &ch, eulerAngle.y * 0.5f);

	ans.x = ch * sp * cb + sh * cp * sb;
	ans.y = -ch * sp * sb + sh * cp * cb;
	ans.z = -sh * sp * cb + ch * cp * sb;
	ans.w = ch * cp * cb + sh * sp * sb;

	return ans;
}

Vector3 MCB::Quaternion::GetRotationAxis(const Quaternion& q)
{
	float sinThetaOver2Sq = 1.0f - q.w * q.w;

	if (sinThetaOver2Sq <= 0.0f)
	{
		return Vector3(0, 0, 1);
	}


	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);


	return Vector3(
		q.x * oneOverSinThetaOver2,
		q.y * oneOverSinThetaOver2,
		q.z * oneOverSinThetaOver2
	);

}

void MCB::Quaternion::GetRotationAxis(const Quaternion& q, Vector3& AxisVec)
{
	float sinThetaOver2Sq = 1.0f - q.w * q.w;

	if (sinThetaOver2Sq <= 0.0f)
	{
		return;
	}


	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);


	AxisVec = Vector3(
		q.x * oneOverSinThetaOver2,
		q.y * oneOverSinThetaOver2,
		q.z * oneOverSinThetaOver2
	);
}

float MCB::Quaternion::GetAngle(const Quaternion& q)
{
	float thetaOver2 = SafeAcos(q.w);
	return thetaOver2 * 2.0f;
}

float MCB::Quaternion::SafeAcos(float a)
{
	if (a <= -1.0f)
	{
		return PI;
	}
	if (a >= 1.0f)
	{
		return 0.0f;
	}
	return acos(a);
}

bool MCB::Quaternion::operator==(const Quaternion& q)
{
	if (q.x == x && q.y == y && q.z == z && q.w == w)
	{
		return true;
	}
	return false;
}


Quaternion MCB::Quaternion::operator-()
{
	return { -x,-y,-z,-w };
}

Quaternion MCB::Quaternion::operator*(float k)
{
	return Quaternion(x * k, y * k, z * k, w * k);
}

Quaternion MCB::Quaternion::operator+(Quaternion q)
{
	return Quaternion(x + q.x, y + q.y, z + q.z, w + q.w);
}

Quaternion MCB::operator*(float k, Quaternion q)
{
	return q * k;
}

Quaternion MCB::Quaternion::Identity()
{
	return Quaternion(0, 0, 0, 1);
}


MCB::Quaternion MCB::Quaternion::Slerp(Quaternion start, const Quaternion& end,
	int32_t time, int32_t maxTime)
{
	float Time = (float)time / (float)maxTime;
	Quaternion ans;
	Quaternion startDemo = start;
	float dot;
	bool isNan = false;
	float angle = GetAngle(start, end, dot, isNan);
	if (isNan)
	{
		startDemo = -start;
	}
	if (dot >= 1.0f - FLT_EPSILON)
	{
		ans.x = (1.f - Time) * startDemo.x + Time * end.x;
		ans.y = (1.f - Time) * startDemo.y + Time * end.y;
		ans.z = (1.f - Time) * startDemo.z + Time * end.z;
		ans.w = (1.f - Time) * startDemo.w + Time * end.w;
		ans.Normalize();
		return ans;
	}

	float st = sinf(angle);

	if (st == 0)
	{
		return -start;
	}

	float sut = sinf(angle * Time);
	float sout = sinf(angle * (1.0f - Time));

	float coeff1 = sout / st;
	float coeff2 = sut / st;

	ans.x = coeff1 * start.x + coeff2 * end.x;
	ans.y = coeff1 * start.y + coeff2 * end.y;
	ans.z = coeff1 * start.z + coeff2 * end.z;
	ans.w = coeff1 * start.w + coeff2 * end.w;

	ans.Normalize();
	return ans;
}


MCB::Quaternion MCB::Quaternion::Slerp(Quaternion start, Quaternion end, float time)//係数を直で入力する用
{
	if (start.operator== (end))
	{
		return start;
	}
	Quaternion ans;
	float dot;
	Quaternion endDemo = end;
	bool isNan = false;
	float angle = GetAngle(start, end, dot, isNan);
	if (isNan)
	{
		endDemo = -end;
	}

	if (dot >= 1.0f - FLT_EPSILON)
	{
		ans.x = (1.f - time) * start.x + time * endDemo.x;
		ans.y = (1.f - time) * start.y + time * endDemo.y;
		ans.z = (1.f - time) * start.z + time * endDemo.z;
		ans.w = (1.f - time) * start.w + time * endDemo.w;
		ans.Normalize();
		return ans;
	}

	float st = sinf(angle);

	if (st == 0)
	{
		return -start;
	}

	float sut = sinf(angle * time);
	float sout = sinf(angle * (1.0f - time));

	float coeff1 = sout / st;
	float coeff2 = sut / st;

	ans.x = coeff1 * start.x + coeff2 * endDemo.x;
	ans.y = coeff1 * start.y + coeff2 * endDemo.y;
	ans.z = coeff1 * start.z + coeff2 * endDemo.z;
	ans.w = coeff1 * start.w + coeff2 * endDemo.w;

	ans.Normalize();
	return ans;

}

Quaternion MCB::SetRota(const Vector3& vec, float angle)
{
	return Quaternion(vec, angle);
}
