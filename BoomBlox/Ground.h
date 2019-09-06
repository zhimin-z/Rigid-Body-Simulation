#ifndef GROUND_H
#define GROUND_H

#include "RigidBody.h"

// The rigid body representing the immovable ground.
// This file is totally boring.
class Ground : public RigidBody
{
public:
	Ground();

	virtual Ground* Clone() const;

	virtual Vector3 GetMinExtents() const;
	virtual Vector3 GetMaxExtents() const;
	virtual Vector3 GetNormalLocalInertialTensor() const;
	virtual Vector3 GetNormalLocalInverseInertialTensor() const;
	virtual float GetNormalMass() const;
	virtual float GetNormalInverseMass() const;
	virtual void DoRender() const;
	virtual bool HasInfiniteMass() const;
};

#endif