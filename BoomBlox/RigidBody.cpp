#include "RigidBody.h"
#include <limits>
#include "GL\freeglut.h"
#include "Material.h"
#include <TestHarness.h>

namespace
{
	float inf = std::numeric_limits<float>::infinity();
}

RigidBody::RigidBody() :
	m_transformation(Matrix4::IDENTITY),
	m_velocity(Vector3::ZERO),
	m_angularVelocity(Vector3::ZERO),
	m_material(NULL),
	m_queuedDeltaVelocity(Vector3::ZERO),
	m_queuedDeltaAngularVelocity(Vector3::ZERO),
	m_hasInfiniteMass(false)
{
}

Material const* RigidBody::GetMaterial() const
{
	return m_material;
}

void RigidBody::SetMaterial(Material const* material)
{
	m_material = material;
}

Vector3 RigidBody::GetNormalLocalInverseInertialTensor() const
{
	Vector3 tensor = GetNormalLocalInertialTensor();
	return Vector3(1/tensor.x, 1/tensor.y, 1/tensor.z);
}

Vector3 RigidBody::GetPosition() const
{
	return m_transformation * Vector3::ZERO;
}

void RigidBody::SetPosition(Vector3 const& position)
{
	m_transformation.setTrans(position);
}

Quaternion RigidBody::GetOrientation() const
{
	return m_transformation.extractQuaternion();
}

void RigidBody::SetOrientation(Quaternion const& orientation)
{
	m_transformation.makeTransform(GetPosition(), Vector3::UNIT_SCALE, orientation);
}


Matrix4 const& RigidBody::GetTransformation() const
{
	return m_transformation;
}

void RigidBody::SetTransformation(Matrix4 const& transformation)
{
	m_transformation = transformation;
}

Vector3 const& RigidBody::GetVelocity() const
{
	return m_velocity;
}

void RigidBody::SetVelocity(Vector3 const& velocity)
{
	m_velocity = velocity;
}

Ogre::Vector3 RigidBody::GetVelocityAtPoint(Vector3 const& p) const
{
	Vector3 v = p-GetPosition();
	return GetAngularVelocity().crossProduct(v) + GetVelocity();
}

Vector3 const& RigidBody::GetAngularVelocity() const
{
	return m_angularVelocity;
}

void RigidBody::SetAngularVelocity(Vector3 const& angularVelocity)
{
	m_angularVelocity = angularVelocity;
}

void RigidBody::QueueImpulse(Vector3 const& impulse, Vector3 const& p)
{
	if(HasInfiniteMass()) return;

	Vector3 deltaVelocity = impulse * GetInverseMass();
	m_queuedDeltaVelocity += deltaVelocity;

	Vector3 deltaOmega = GetInverseInertialTensor() * (p-GetPosition()).crossProduct(impulse);
	m_queuedDeltaAngularVelocity += deltaOmega;
}

bool RigidBody::ApplyQueuedImpulses()
{
	if(HasInfiniteMass()) return false;

	bool significant = (m_queuedDeltaVelocity.squaredLength() > 1e-8 || m_queuedDeltaAngularVelocity.squaredLength() > 1e-8);

	if(significant)
	{
		SetVelocity(GetVelocity() + m_queuedDeltaVelocity);
		SetAngularVelocity(GetAngularVelocity() + m_queuedDeltaAngularVelocity);
	}


	m_queuedDeltaVelocity = Vector3::ZERO;
	m_queuedDeltaAngularVelocity = Vector3::ZERO;

	return significant;
}

void RigidBody::ApplyImpulse(Vector3 const& impulse, Vector3 const& point)
{
	if(HasInfiniteMass()) return;

	Vector3 deltaVelocity = impulse * GetInverseMass();
	SetVelocity(GetVelocity() + deltaVelocity);

	Vector3 deltaOmega = GetInverseInertialTensor() * (point-GetPosition()).crossProduct(impulse);
	SetAngularVelocity(GetAngularVelocity() + deltaOmega);
}

void RigidBody::SaveTransformation()
{
	m_savedTransformation = m_transformation;
}

void RigidBody::RestoreTransformation()
{
	m_transformation = m_savedTransformation;
}

void RigidBody::SaveVelocity()
{
	m_savedVelocity = m_velocity;
	m_savedAngularVelocity = m_angularVelocity;
}

void RigidBody::RestoreVelocity()
{
	m_velocity = m_savedVelocity;
	m_angularVelocity = m_savedAngularVelocity;
}

void RigidBody::AdvanceTransformation(float dT)
{
	Vector3 newPosition = GetPosition() + GetVelocity()*dT;
	float angularSpeed = GetAngularVelocity().length();
	Vector3 axis = GetAngularVelocity().normalisedCopy();
	Quaternion newOrientation = Quaternion(angularSpeed * dT, axis) * GetOrientation();
	newOrientation.normalise();

	m_transformation.makeTransform(newPosition, Vector3::UNIT_SCALE, newOrientation);
}

void RigidBody::AdvanceVelocity(Vector3 const& F, float dT)
{
	if(!HasInfiniteMass())
	{
		SetVelocity(GetVelocity() + F * dT);
	}
}

void RigidBody::SetInfiniteMass()
{
	m_hasInfiniteMass = true;
}

void RigidBody::UnsetInfiniteMass()
{
	m_hasInfiniteMass = false;
}

bool RigidBody::HasInfiniteMass() const
{
	return m_hasInfiniteMass;
}

Vector3 RigidBody::GetLocalInertialTensor() const
{
	return HasInfiniteMass() ? Vector3(inf, inf, inf) : GetNormalLocalInertialTensor();
}

Vector3 RigidBody::GetLocalInverseInertialTensor() const
{
	return HasInfiniteMass() ? Vector3(0, 0, 0) : GetNormalLocalInverseInertialTensor();
}

float RigidBody::GetMass() const
{
	return HasInfiniteMass() ? inf : GetNormalMass();
}

float RigidBody::GetInverseMass() const
{
	return HasInfiniteMass() ? inf : GetNormalInverseMass();
}

Matrix3 RigidBody::GetInertialTensor() const
{
	Vector3 diagonal = GetLocalInertialTensor();
	Matrix3 localTensor = Matrix3::ZERO;
	localTensor[0][0] = diagonal.x;
	localTensor[1][1] = diagonal.y;
	localTensor[2][2] = diagonal.z;

	Matrix3 rotation;
	GetTransformation().extract3x3Matrix(rotation);

	return rotation * localTensor * rotation.Transpose();
}

Matrix3 RigidBody::GetInverseInertialTensor() const
{
	Vector3 diagonal = GetLocalInverseInertialTensor();
	Matrix3 localInverseTensor = Matrix3::ZERO;
	localInverseTensor[0][0] = diagonal.x;
	localInverseTensor[1][1] = diagonal.y;
	localInverseTensor[2][2] = diagonal.z;

	Matrix3 rotation;
	GetTransformation().extract3x3Matrix(rotation);

	return rotation.Transpose() * localInverseTensor * rotation;
}

void RigidBody::Render() const
{
	glColor3f(GetMaterial()->color.x, GetMaterial()->color.y, GetMaterial()->color.z);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	
	Vector3 pos = GetPosition();
	glTranslatef(pos.x, pos.y, pos.z);
	float angle;
	Vector3 axis;
	GetOrientation().ToAngleAxis(angle, axis);
	glRotatef(angle*180.0f/M_PI, axis.x, axis.y, axis.z);

	DoRender();
	glPopMatrix();
	int err = glGetError();
	err = err;
}
