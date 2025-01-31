#pragma once
#include <Vector3.h>

namespace MCBM
{
	class Matrix3x3
	{
	public:
		double mat[9];
		Matrix3x3 Mat3Identity();
		Matrix3x3 ProjectionMatrix(const Vector3& d);
		Matrix3x3 Mat3Add(const Matrix3x3& A, const Matrix3x3& B);
		Matrix3x3 Mat3Scale(const Matrix3x3& A, double s);
		Vector3 Mat3Mulvec(const Matrix3x3& A, const Vector3& v);
		bool Invert3x3(const Matrix3x3& A, Matrix3x3& Ainv);
	};
}

