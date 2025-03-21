#include "SphereCollider.h"

MCB::SphereCollider::SphereCollider(const Vector3D& offset,float radius)
{
	offset_ = offset,radius_ = radius; primitive_ = PrimitiveType::SPHERE;
};

void MCB::SphereCollider::Update()
{
	DirectX::XMMATRIX mat = object3d_->GetMatWorld();
	Sphere::centerPosition_.vec_.x_ = mat.r[3].m128_f32[0] + offset_.vec_.x_;
	Sphere::centerPosition_.vec_.y_ = mat.r[3].m128_f32[1] + offset_.vec_.y_;
	Sphere::centerPosition_.vec_.z_ = mat.r[3].m128_f32[2] + offset_.vec_.z_;
	Sphere::radius_ = radius_;
	GetObject3D()->hited_ = false;
}

void MCB::SphereCollider::SetRadius(float radius)
{
	radius_ = radius;
}

float MCB::SphereCollider::GetRaius()
{
	return radius_;
}
