#ifndef SPHERE_H
#define SPHERE_H

#include "RigidBody.h"

class Sphere : public RigidBody
{
public:
	Sphere(float radius);

	virtual Sphere* Clone() const;

	virtual Vector3 GetMinExtents() const;
	virtual Vector3 GetMaxExtents() const;
	virtual Vector3 GetNormalLocalInertialTensor() const;
	virtual float GetNormalMass() const;
	virtual void DoRender() const;

	float GetRadius() const;

private:
	float m_radius;
	float m_density;
};

#endif