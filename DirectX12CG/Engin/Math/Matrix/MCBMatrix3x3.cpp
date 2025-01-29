#include <MCBMatrix3x3.h>
#include <cmath>

using namespace MCB;
// ---------- 行列演算 ----------
Matrix3x3 Matrix3x3::Mat3Identity()
{
	Matrix3x3 A;
	A.mat[ 0 ] = 1; A.mat[ 1 ] = 0; A.mat[ 2 ] = 0;
	A.mat[ 3 ] = 0; A.mat[ 4 ] = 1; A.mat[ 5 ] = 0;
	A.mat[ 6 ] = 0; A.mat[ 7 ] = 0; A.mat[ 8 ] = 1;
	return A;
}

// A = I - d d^T (dは3次元ベクトル)
Matrix3x3 Matrix3x3::ProjectionMatrix(const Vector3D& d) {
	// dは単位ベクトルを想定
	// I - d*d^T
	Matrix3x3 P;
	P.mat[ 0 ] = 1.0 - d.vec_.x_ * d.vec_.x_;
	P.mat[ 1 ] = -d.vec_.x_ * d.vec_.y_;
	P.mat[ 2 ] = -d.vec_.x_ * d.vec_.z_;

	P.mat[ 3 ] = -d.vec_.y_ * d.vec_.x_;
	P.mat[ 4 ] = 1.0 - d.vec_.y_ * d.vec_.y_;
	P.mat[ 5 ] = -d.vec_.y_ * d.vec_.z_;

	P.mat[ 6 ] = -d.vec_.z_ * d.vec_.x_;
	P.mat[ 7 ] = -d.vec_.z_ * d.vec_.y_;
	P.mat[ 8 ] = 1.0 - d.vec_.z_ * d.vec_.z_;

	return P;
}

// C = A + B (行列の要素ごとに加算)
Matrix3x3 Matrix3x3::Mat3Add(const Matrix3x3& A,const Matrix3x3& B) {
	Matrix3x3 C;
	for ( int i = 0; i < 9; i++ )
	{
		C.mat[ i ] = A.mat[ i ] + B.mat[ i ];
	}
	return C;
}

// C = s*A (行列Aのスカラー倍)
Matrix3x3 Matrix3x3::Mat3Scale(const Matrix3x3& A,double s) {
	Matrix3x3 C;
	for ( int i = 0; i < 9; i++ )
	{
		C.mat[ i ] = s * A.mat[ i ];
	}
	return C;
}

// v2 = A * v1 (3×3行列×3ベクトル)
Vector3D Matrix3x3::Mat3Mulvec(const Matrix3x3& A,const Vector3D& v) {
	return Vector3D(
		A.mat[ 0 ] * v.vec_.x_ + A.mat[ 1 ] * v.vec_.y_ + A.mat[ 2 ] * v.vec_.z_,
		A.mat[ 3 ] * v.vec_.x_ + A.mat[ 4 ] * v.vec_.y_ + A.mat[ 5 ] * v.vec_.z_,
		A.mat[ 6 ] * v.vec_.x_ + A.mat[ 7 ] * v.vec_.y_ + A.mat[ 8 ] * v.vec_.z_
	);
}

// 3×3行列を (ガウス消去等で) 逆行列にする。
// 成功したらtrue, 失敗したらfalse (特異行列)。
bool Matrix3x3::Invert3x3(const Matrix3x3& A,Matrix3x3& Ainv)
{
	// row-majorで計算しやすいように要素を一旦取り出す
	double a11 = A.mat[ 0 ],a12 = A.mat[ 1 ],a13 = A.mat[ 2 ];
	double a21 = A.mat[ 3 ],a22 = A.mat[ 4 ],a23 = A.mat[ 5 ];
	double a31 = A.mat[ 6 ],a32 = A.mat[ 7 ],a33 = A.mat[ 8 ];

	// 行列式
	double det = a11 * ( a22 * a33 - a23 * a32 )
		- a12 * ( a21 * a33 - a23 * a31 )
		+ a13 * ( a21 * a32 - a22 * a31 );

	if ( std::fabs(det) < 1.0e-15 )
	{
		return false;
	}
	double invDet = 1.0 / det;

	// 余因子行列^Tで求める
	Ainv.mat[ 0 ] = ( a22 * a33 - a23 * a32 ) * invDet;
	Ainv.mat[ 1 ] = -( a12 * a33 - a13 * a32 ) * invDet;
	Ainv.mat[ 2 ] = ( a12 * a23 - a13 * a22 ) * invDet;

	Ainv.mat[ 3 ] = -( a21 * a33 - a23 * a31 ) * invDet;
	Ainv.mat[ 4 ] = ( a11 * a33 - a13 * a31 ) * invDet;
	Ainv.mat[ 5 ] = -( a11 * a23 - a13 * a21 ) * invDet;

	Ainv.mat[ 6 ] = ( a21 * a32 - a22 * a31 ) * invDet;
	Ainv.mat[ 7 ] = -( a11 * a32 - a12 * a31 ) * invDet;
	Ainv.mat[ 8 ] = ( a11 * a22 - a12 * a21 ) * invDet;

	return true;
}