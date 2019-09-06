#ifndef BOX_H
#define BOX_H

#include "RigidBody.h"

class Box : public RigidBody
{
public:
	Box(Vector3 const& halfSize);

	virtual Box* Clone() const;

	// Get the signed extents along the world-space plane equation.
	// Particularly speaking: Return as [a,b] the interval such that for any a < x < b, there
	// is a point in this rigid body which is distance x from the plane defined by v.
	//
	// This is calculable for any rigid body, but is currently not defined except for 
	// boxes because it is currently only used by boxes.
	void GetExtentsAlongAxis(Vector4 const& v, float & a, float & b) const;

	// Get the closest point in/on the box to an arbitrary worldspace point.
	Vector3 GetClosestPoint(Vector3 const& p) const;

	virtual Vector3 GetMinExtents() const;
	virtual Vector3 GetMaxExtents() const;

	// Get the local (diagonal) intertial tensor when the object doesn't have infinite mass.
	virtual Vector3 GetNormalLocalInertialTensor() const;

	// Get normal (non-infinite) mass
	virtual float GetNormalMass() const;

	virtual void DoRender() const;

	Vector3 const& GetHalfSize() const;

private:
	Vector3 m_halfSize;
	float longestHalfSize;
};

#endif