#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <algorithm>
#include <OgreMath.h>

struct Material;

class RigidBody
{
public:
	RigidBody();

	// Virtual clone method.
	virtual RigidBody* Clone() const = 0;

	// Get/set the current material, which is used for density and other material calculations, as well as render color.
	Material const* GetMaterial() const;
	void SetMaterial(Material const* material);

	// Get the minimum and maximum extents of the object, in world space. These will be affected by rigid transformations.
	virtual Vector3 GetMinExtents() const = 0;
	virtual Vector3 GetMaxExtents() const = 0;

	// Get the diagonal elements of the inertial tensor of the object, in object space (when not set to infinite mass).
	// Since all our objects are axis-aligned in object space, this completely typifies the tensor (all other elements are zero).
	virtual Vector3 GetNormalLocalInertialTensor() const = 0;
	// Get the (element-wise) inverse of the above. Note that there is a default implementation, which uses GetNormalLocalInertialTensor.
	virtual Vector3 GetNormalLocalInverseInertialTensor() const;

	// Get the mass (when not explicitly set to infinite mass).
	virtual float GetNormalMass() const = 0;
	// Get the inverse mass (when not explicitly set to infinite mass).
	virtual float GetNormalInverseMass() const { return 1.0f / GetNormalMass(); }


	// Versions of the above which respect SetInfiniteMass().
	Vector3 GetLocalInertialTensor() const;
	Vector3 GetLocalInverseInertialTensor() const;
	float GetMass() const;
	float GetInverseMass() const;

	// Get the inertial tensor in world space
	virtual Matrix3 GetInertialTensor() const;
	virtual Matrix3 GetInverseInertialTensor() const;


	Vector3 GetPosition() const;
	void SetPosition(Vector3 const& position);

	Quaternion GetOrientation() const;
	void SetOrientation(Quaternion const& orientation);

	Matrix4 const& GetTransformation() const;
	void SetTransformation(Matrix4 const& transformation);

	Vector3 const& GetVelocity() const;
	void SetVelocity(Vector3 const& velocity);

	Vector3 GetVelocityAtPoint(Vector3 const& p) const;

	// Angular velocity is represented as the unit-length axis of rotation, multiplied 
	// by the magnitude of the rotation in radians per second.
	Vector3 const& GetAngularVelocity() const;
	void SetAngularVelocity(Vector3 const& angularVelocity);

	// queues an impulse to the body, which will change its velocity.
	// "impulse" is the delta-momentum vector, in world-space.
	// "point" is the world-space point at which the impulse is applied.
	// Queueing impulses frees us from having to worry about collision processing orders,
	// at least in one respect.
	void QueueImpulse(Vector3 const& impulse, Vector3 const& point);

	// Applies all queued impulses. Returns true if this caused any significant effects; false otherwise
	bool ApplyQueuedImpulses();

	// an immediate version of QueueImpulse.
	void ApplyImpulse(Vector3 const& impulse, Vector3 const& point);

	// Save positional and orientational information, so that hypothetical things can be done to the object.
	void SaveTransformation();
	// Restores saved info.
	void RestoreTransformation();

	// The same as the above, but for velocity.
	void SaveVelocity();
	void RestoreVelocity();

	// One half of a forward Euler step.
	void AdvanceTransformation(float dT);

	// The other half of a forward Euler step.
	void AdvanceVelocity(Vector3 const& F, float dT);

	// causes AddImpulse to have no effect. Useful for temporarily immobilizing bodies to force
	// all the momentum of a collision to be applied to a single body.
	void SetInfiniteMass();
	void UnsetInfiniteMass();
	virtual bool HasInfiniteMass() const; // can be overloaded to force infinite mass on.

	// Render to the OpenGL window, including transformations and color, but not including lighting properties.
	void Render() const;

protected:
	// perform geometry rendering (but not color setup or transformation)
	virtual void DoRender() const = 0;

private:
	Matrix4 m_transformation;
	Matrix4 m_savedTransformation;
	Vector3 m_velocity;
	Vector3 m_angularVelocity;
	Vector3 m_savedVelocity;
	Vector3 m_savedAngularVelocity;
	Vector3 m_queuedDeltaVelocity;
	Vector3 m_queuedDeltaAngularVelocity;

	Material const* m_material;

	bool m_hasInfiniteMass;
};

#endif