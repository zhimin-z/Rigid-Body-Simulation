#include "Sphere.h"
#include "Material.h"
#include "GL\freeglut.h"

Sphere::Sphere(float radius) : m_radius(radius)
{

}

Sphere* Sphere::Clone() const
{
	return new Sphere(*this);
}

Vector3 Sphere::GetMinExtents() const
{
	return Vector3(-GetRadius(), -GetRadius(), -GetRadius()) + GetPosition();
}

Vector3 Sphere::GetMaxExtents() const
{
	return Vector3(GetRadius(), GetRadius(), GetRadius()) + GetPosition();
}

Vector3 Sphere::GetNormalLocalInertialTensor() const
{
	return Vector3::UNIT_SCALE * (0.4f * GetNormalMass() * GetRadius() * GetRadius());
}

float Sphere::GetNormalMass() const
{
	return GetMaterial()->density * (4.0f/3.0f) * float(M_PI) * GetRadius() * GetRadius();
}

//float Sphere::SignedDistance(Vector3 const& point) const
//{
//	float distFromCenter = (point - GetPosition()).length();
//	return distFromCenter - GetRadius();
//}

void Sphere::DoRender() const
{
	glutSolidSphere(GetRadius(), 20, 20);
}

float Sphere::GetRadius() const
{
	return m_radius;
}
