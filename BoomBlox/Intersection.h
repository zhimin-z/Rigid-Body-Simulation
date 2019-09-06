#ifndef INTERSECTION_H
#define INTERSECTION_H

#include <OgreMath.h>
#include <vector>

class RigidBody;

// An interpenetration between two bodies.
// Interpenetrations (almost) always come in pairs, one for each body
struct Intersection
{
	Intersection(RigidBody* bodyA, RigidBody* bodyB, Vector3 contactPoint, Vector3 outVector);
	RigidBody* bodyA; // this body
	RigidBody* bodyB; // the body this penetrates into
	Vector3 contactPoint; // the point where contact will be applied (in world space)
	Vector3 contactPointA; // the same point in bodyA space
	Vector3 contactPointB; // the same point in bodyB space
	Vector3 outVector; // the vector of restitution for BodyA (in world space)
};

// sees if any point in "body" is inside "otherBody". If so, it adds the intersection to "intersections".
void FindIntersection(RigidBody * bodyA, RigidBody * bodyB, std::vector<Intersection> & intersections);
Vector4 TransformPlane(Vector4 plane, Matrix4 transform);
void BubbleSort (int *u, float *r, int n);
#endif