#pragma once
#include <Vector3D.h>

namespace MCB
{
	class Matrix3x3
	{
	public:
		double mat[ 9 ];
		Matrix3x3 Mat3Identity();
		Matrix3x3 ProjectionMatrix(const Vector3D& d);
		Matrix3x3 Mat3Add(const Matrix3x3& A,const Matrix3x3& B);
		Matrix3x3 Mat3Scale(const Matrix3x3& A,double s);
		Vector3D Mat3Mulvec(const Matrix3x3& A,const Vector3D& v);
		bool Invert3x3(const Matrix3x3& A,Matrix3x3& Ainv);
	};
}

