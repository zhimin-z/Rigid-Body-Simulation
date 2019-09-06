#include "Ground.h"
#include <limits>
#include "GL\freeglut.h"
#include "Material.h"

namespace
{
	float inf = std::numeric_limits<float>::infinity();
}

Ground::Ground()
{
	SetPosition(Vector3::ZERO);
	SetOrientation(Quaternion::IDENTITY);
	SetVelocity(Vector3::ZERO);
	SetAngularVelocity(Vector3::ZERO);
}

Ground* Ground::Clone() const
{
	return new Ground(*this);
}

Vector3 Ground::GetMinExtents() const
{
	return Vector3(-inf, -inf, -inf);
}

Vector3 Ground::GetMaxExtents() const
{
	return Vector3(inf, 0, inf);
}

Vector3 Ground::GetNormalLocalInertialTensor() const
{
	return Vector3(inf, inf, inf);
}

Vector3 Ground::GetNormalLocalInverseInertialTensor() const
{
	return Vector3(0, 0, 0);
}

float Ground::GetNormalMass() const
{
	return inf;
}

float Ground::GetNormalInverseMass() const
{
	return 0;
}

void Ground::DoRender() const
{
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
	glBegin(GL_POLYGON);
	glVertex3f(-1000, 0, -1000);
	glVertex3f( 1000, 0, -1000);
	glVertex3f( 1000, 0,  1000);
	glVertex3f(-1000, 0,  1000);
	glEnd();
	glPopAttrib();
}

bool Ground::HasInfiniteMass() const
{
	// LOOK Even when not immobilized, mass is infinite.
	return true;
}
