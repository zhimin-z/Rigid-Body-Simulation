#include "Intersection.h"
#include "Box.h"
#include "Ground.h"
#include "Sphere.h"
#include "Material.h"
#include <cassert>
#include <TestHarness.h>
#include <vector>
#include <algorithm>
#define error 1e-3f

namespace
{
	void FindIntersectionBox(Box * bodyA, RigidBody * bodyB, std::vector<Intersection> & intersections);
	void FindIntersectionGround(Ground * bodyA, RigidBody * bodyB, std::vector<Intersection> & intersections);
	void FindIntersectionSphere(Sphere * bodyA, RigidBody * bodyB, std::vector<Intersection> & intersections);

	void FindIntersectionBoxBox(Box * box, Box * box2, std::vector<Intersection> & intersections);
	void FindIntersectionBoxGround(Box * box, Ground * ground, std::vector<Intersection> & intersections);
	void FindIntersectionBoxSphere(Box * box, Sphere * sphere, std::vector<Intersection> & intersections);
	void FindIntersectionGroundSphere(Ground * ground, Sphere * sphere, std::vector<Intersection> & intersections);
	void FindIntersectionSphereSphere(Sphere * sphere, Sphere * sphere2, std::vector<Intersection> & intersections);

	// returns the amount of overlap between the intervals [a,b] and [c,d]
	float ComputeOverlap(float a, float b, float c, float d);

	void AddSegmentPlaneIntersection(Vector3 const& L0, Vector3 const& L1, Vector4 const& P, std::vector<Vector3> & intersections);
	float ComputeOverlapCentroid(float a, float b, float c, float d);
	void ComputeBoundingBox(std::vector<Vector3> const& points, Vector3 & min, Vector3 & max);

	const Vector4 axes4[3] = { 
		Vector4(1,0,0,0),
		Vector4(0,1,0,0),
		Vector4(0,0,1,0),
	};

	const Vector3 axes3[3] = { 
		Vector3(1,0,0),
		Vector3(0,1,0),
		Vector3(0,0,1),
	};
}

Intersection::Intersection(RigidBody* bodyA, RigidBody* bodyB, Vector3 contactPoint, Vector3 outVector) :
	bodyA(bodyA), bodyB(bodyB), contactPoint(contactPoint), outVector(outVector)
{
	contactPointA = bodyA->GetTransformation().inverseAffine()*contactPoint;
	contactPointB = bodyB->GetTransformation().inverseAffine()*contactPoint;
}

//LOOK Uses a variant of double-dispatch which is not particularly optimized, but easy to understand,
//extend, and debug. 
void FindIntersection(RigidBody * bodyA, RigidBody * bodyB, std::vector<Intersection> & intersections)
{
	Box * box = dynamic_cast<Box *>(bodyA);
	if(box)
	{
		FindIntersectionBox(box, bodyB, intersections);
		return;
	}
	
	Ground * ground = dynamic_cast<Ground *>(bodyA);
	if(ground)
	{
		FindIntersectionGround(ground, bodyB, intersections);
		return;
	}

	Sphere * sphere = dynamic_cast<Sphere *>(bodyA);
	if(sphere)
	{
		FindIntersectionSphere(sphere, bodyB, intersections);
		return;
	}

	// if we get here, there's an object type we don't know how to handle
	assert(false);
}

namespace
{
	void FindIntersectionBox(Box * box, RigidBody * bodyB, std::vector<Intersection> & intersections)
	{
		Box * box2 = dynamic_cast<Box *>(bodyB);
		if(box2)
		{
			FindIntersectionBoxBox(box, box2, intersections);
			return;
		}

		Ground * ground = dynamic_cast<Ground *>(bodyB);
		if(ground)
		{
			FindIntersectionBoxGround(box, ground, intersections);
			return;
		}

		Sphere * sphere = dynamic_cast<Sphere *>(bodyB);
		if(sphere)
		{
			FindIntersectionBoxSphere(box, sphere, intersections);
			return;
		}
		// if we get here, there's an object type we don't know how to handle
		assert(false);
	}

	void FindIntersectionGround(Ground * ground, RigidBody * bodyB, std::vector<Intersection> & intersections)
	{
		Box * box = dynamic_cast<Box *>(bodyB);
		if(box)
		{
			FindIntersectionBoxGround(box, ground, intersections);
			return;
		}

		Ground * ground2 = dynamic_cast<Ground *>(bodyB);
		if(ground2)
		{
			// um... there's only one ground.
			// the only way to get here is if we've decided to collide objects with themselves.
			// that's the sort of thing we should know about.
			assert(false);
			return;
		}

		Sphere * sphere = dynamic_cast<Sphere *>(bodyB);
		if(sphere)
		{
			FindIntersectionGroundSphere(ground, sphere, intersections);
			return;
		}

		// if we get here, there's an object type we don't know how to handle
		assert(false);
	}

	void FindIntersectionSphere(Sphere * sphere, RigidBody * bodyB, std::vector<Intersection> & intersections)
	{
		Box * box = dynamic_cast<Box *>(bodyB);
		if(box)
		{
			FindIntersectionBoxSphere(box, sphere, intersections);
			return;
		}

		Ground * ground = dynamic_cast<Ground *>(bodyB);
		if(ground)
		{
			FindIntersectionGroundSphere(ground, sphere, intersections);
			return;
		}

		Sphere * sphere2 = dynamic_cast<Sphere *>(bodyB);
		if(sphere2)
		{
			FindIntersectionSphereSphere(sphere, sphere2, intersections);
			return;
		}

		// if we get here, there's an object type we don't know how to handle
		assert(false);
	}

	void FindBestAxis( Box * boxA, Box * boxB, float &bestAxisOverlap, int &bestAxis, bool &upperOverlap )
	{
		Matrix4 transform = boxA->GetTransformation();

		for(int i=0; i<3; i++)
		{
			// separating axis test along axis i of box A, with plane containing center of box A
			float a,b,c,d;
			b = boxA->GetHalfSize()[i];
			a = -b;

			boxB->GetExtentsAlongAxis(TransformPlane(axes4[i], transform), c, d);

			float overlap = ComputeOverlap(a,b,c,d);

			if(i==0 || overlap < bestAxisOverlap)
			{
				bestAxisOverlap = overlap;
				bestAxis = i;
				upperOverlap = ((c+d) > (a+b));
			}

			if(overlap == 0) // hey! No intersection at all!
			{
				return;
			}
		}
	}

	void IntersectBoundingBox(Vector3 & min, Vector3 & max, Vector3 const& oMin, Vector3 const& oMax)
	{
		min.makeCeil(oMin);
		max.makeFloor(oMax);
	}

	struct PotentialPoints
	{
		PotentialPoints(Vector3 const& p) : p(p) { }
		Vector3 p;
		float speed;
		bool operator<(PotentialPoints const& other) { return speed < other.speed; }
	};

	Vector3 GetBestOverlapPoint(Box* penetrated, int axis, bool top, Box* penetrator)
	{
		Vector4 p = axes4[axis] * (top?1.0f:-1.0f);
		p.w = -penetrated->GetHalfSize()[axis];
		Vector4 pb = TransformPlane(p, penetrator->GetTransformation().inverseAffine()*penetrated->GetTransformation());

		Vector3 hs = penetrator->GetHalfSize();
		const Vector3 vertices[8] =
		{
			Vector3(-hs.x,-hs.y,-hs.z), //0
			Vector3( hs.x,-hs.y,-hs.z), //1
			Vector3(-hs.x, hs.y,-hs.z), //2
			Vector3( hs.x, hs.y,-hs.z), //3
			Vector3(-hs.x,-hs.y, hs.z), //4
			Vector3( hs.x,-hs.y, hs.z), //5
			Vector3(-hs.x, hs.y, hs.z), //6
			Vector3( hs.x, hs.y, hs.z)  //7
		};

		std::vector<Vector3> intersections;

		AddSegmentPlaneIntersection(vertices[0], vertices[1], pb, intersections);
		AddSegmentPlaneIntersection(vertices[0], vertices[2], pb, intersections);
		AddSegmentPlaneIntersection(vertices[0], vertices[4], pb, intersections);
		AddSegmentPlaneIntersection(vertices[1], vertices[3], pb, intersections);
		AddSegmentPlaneIntersection(vertices[1], vertices[5], pb, intersections);
		AddSegmentPlaneIntersection(vertices[2], vertices[3], pb, intersections);
		AddSegmentPlaneIntersection(vertices[2], vertices[6], pb, intersections);
		AddSegmentPlaneIntersection(vertices[3], vertices[7], pb, intersections);
		AddSegmentPlaneIntersection(vertices[4], vertices[5], pb, intersections);
		AddSegmentPlaneIntersection(vertices[4], vertices[6], pb, intersections);
		AddSegmentPlaneIntersection(vertices[5], vertices[7], pb, intersections);
		AddSegmentPlaneIntersection(vertices[6], vertices[7], pb, intersections);

		assert(!intersections.empty()); // We don't expect nonintersection

		Matrix4 backTransform = penetrated->GetTransformation().inverseAffine()*penetrator->GetTransformation();

		for(int i=0,n=int(intersections.size()); i<n; i++)
		{
			intersections[i] = backTransform * intersections[i];
		}

		Vector3 min, max;
		ComputeBoundingBox(intersections, min, max);
		IntersectBoundingBox(min, max, -penetrated->GetHalfSize(), penetrated->GetHalfSize());

		std::vector<PotentialPoints> potentialPoints;
		if(fabs(min.x-max.x) < error)
		{
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, min.y, min.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, max.y, min.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, min.y, max.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, max.y, max.z));
		}
		else if(fabs(min.y-max.y) < error)
		{
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, min.y, min.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(max.x, min.y, min.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, min.y, max.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(max.x, min.y, max.z));
		}
		else if(fabs(min.z-max.z) < error)
		{
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, min.y, min.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(max.x, min.y, min.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(min.x, max.y, min.z));
			potentialPoints.push_back(penetrated->GetTransformation()*Vector3(max.x, max.y, min.z));
		}
		else
		{
			assert(false);
		}

		Vector3 axis3 = axes3[axis] * (top?1.0f:-1.0f);

		for(int i=0,n=int(potentialPoints.size()); i<n; i++)
		{
			Vector3 relativeVelocity = 
				penetrator->GetVelocityAtPoint(potentialPoints[i].p) - 
				penetrated->GetVelocityAtPoint(potentialPoints[i].p);
			potentialPoints[i].speed = relativeVelocity.dotProduct(axis3);
		}

		std::sort(potentialPoints.begin(), potentialPoints.end());

		if(potentialPoints[1].speed / potentialPoints[0].speed < 0.9) // one-point contact
		{
			return potentialPoints[0].p;
		}
		else if(potentialPoints[2].speed / potentialPoints[1].speed < 0.9) // two-point contact
		{
			return (potentialPoints[0].p + potentialPoints[1].p) / 2;
		}
		else // four-point contact
		{
			return penetrated->GetTransformation()*((max+min)/2);
		}
	}

	float GetSkew(RigidBody const* a, RigidBody const* b)
	{
		Matrix3 ma, mb;
		a->GetTransformation().extract3x3Matrix(ma);
		b->GetTransformation().extract3x3Matrix(mb);
		Matrix3 r = ma.Inverse()*mb;
		float sum = 0;
		for(int i=0;i<9;i++)
		{
			sum += fabs(r[0][i]);
		}

		return sum-3;
	}

	void AddSegmentPlaneIntersection(Vector3 const& L0, Vector3 const& L1, Vector4 const& P, std::vector<Vector3> & intersections)
	{
		float L0P = Vector4(L0).dotProduct(P);
		float L1P = Vector4(L1).dotProduct(P);

		float denom = L0P-L1P;
		if(fabs(denom) < error)
		{
			return;
		}
		float alpha = L0P / denom;
		// It's better to intersect than not to intersect, because if we find too few intersections we're hosed.
		// This makes things work better for contact.
		if(alpha > -0.001 && alpha < 1.001)
		{
			Vector3 point = (1-alpha)*L0 + alpha*L1;
			intersections.push_back(point);
		}
	}

	void ComputeBoundingBox(std::vector<Vector3> const& points, Vector3 & min, Vector3 & max)
	{
		min = points[0];
		max = points[0];

		for(int i=1, n=int(points.size()); i<n; i++)
		{
			Vector3 const& p = points[i];
			min.makeFloor(p);
			max.makeCeil(p);
		}
	}

	TEST(FindIntersectionBoxBox, Intersection)
	{
		Box a(Vector3(1,2,3));
		Box b(Vector3(4,5,6));
		std::vector<Intersection> i;

		a.SetPosition(Vector3(6,0,0));
		FindIntersection(&a, &b, i);
		CHECK(i.size() == 0);
		FindIntersection(&b, &a, i);
		CHECK(i.size() == 0);

		// axis-aligned collision
		a.SetPosition(Vector3(2,6,0));
		FindIntersection(&b, &a, i);
		CHECK(i.size() == 1);
		CHECK(i.back().bodyA == &a);
		CHECK(i.back().bodyB == &b);
		CHECK(i.back().outVector.positionEquals(Vector3(0,1,0)));
		CHECK(i.back().contactPoint.positionEquals(Vector3(2,5,0)));

		// axis-unaligned collision
		Box c(Vector3(1,1,1));
		c.SetPosition(Vector3(1,6,0));
		c.SetOrientation(Quaternion(float(M_PI_4),Vector3(0,0,1)));
		FindIntersection(&c, &b, i);
		CHECK(i.size() == 2);
		CHECK(i.back().bodyA == &c);
		CHECK(i.back().bodyB == &b);
		CHECK(i.back().outVector.positionEquals(Vector3(0,1,0)));
		CHECK(i.back().contactPoint.positionEquals(Vector3(1,5,0)));
	}

	void FindIntersectionBoxBox(Box * boxA, Box * boxB, std::vector<Intersection> & intersections)
	{
		// the "best axis" is the axis with the least overlap between the two bodies

		// process axes of boxA
		float bestAxisOverlapA;
		bool upperOverlapA;
		int bestAxisA;
		FindBestAxis(boxA, boxB, bestAxisOverlapA, bestAxisA, upperOverlapA);

		if(bestAxisOverlapA < error)
		{
			return;
		}

		// process axes of boxB
		float bestAxisOverlapB;
		bool upperOverlapB;
		int bestAxisB;
		FindBestAxis(boxB, boxA, bestAxisOverlapB, bestAxisB, upperOverlapB);

		if(bestAxisOverlapB < error)
		{
			return;
		}

		// if we get here, there is certainly an intersection (SA test failed)
		bool bestAxisOnA = (bestAxisOverlapA <= bestAxisOverlapB);
		int bestAxis = bestAxisOnA ? bestAxisA : bestAxisB;
		bool upperOverlap = bestAxisOnA ? upperOverlapA : upperOverlapB;
		Box* penetrator = bestAxisOnA ? boxB : boxA;
		Box* penetrated = bestAxisOnA ? boxA : boxB;

		Vector3 outVectorLS = upperOverlap ? axes3[bestAxis] : -axes3[bestAxis];
		Vector3 outVector = penetrated->GetTransformation()*outVectorLS - penetrated->GetPosition();
		Vector3 closestPoint = GetBestOverlapPoint(penetrated, bestAxis, upperOverlap, penetrator);

		if(outVector.squaredLength() > 1.0001)
		{
			__debugbreak();
		}
		intersections.push_back(Intersection(penetrator, penetrated, closestPoint, outVector));
	}

	TEST(FindIntersectionBoxGround, Intersection)
	{
		Box b(Vector3(2,1,2));
		Ground g;
		std::vector<Intersection> i;

		// nonintersecting
		b.SetPosition(Vector3(0,2,0));
		FindIntersection(&b, &g, i);
		CHECK(i.size() == 0);

		// intersecting flatly (4 vertices below ground)
		b.SetPosition(Vector3(0,0,0));
		FindIntersection(&b, &g, i);
		CHECK(i.size() == 1);
		CHECK(i.back().bodyA == &b);
		CHECK(i.back().bodyB == &g);
		CHECK(i.back().outVector.positionEquals(Vector3(0,1,0)));
		FLOATS_EQUAL(0, i.back().contactPoint.x);

		// intersecting on an angle (2 vertices below ground)
		b.SetOrientation(Quaternion(float(M_PI)/4, Vector3::UNIT_Z));
		b.SetPosition(Vector3(0,1,0));
		FindIntersection(&b, &g, i);
		CHECK(i.size() == 2);
		CHECK(i.back().bodyA == &b);
		CHECK(i.back().bodyB == &g);
		CHECK(i.back().outVector.positionEquals(Vector3(0,1,0)));
		FLOATS_EQUAL(-0.707107f, i.back().contactPoint.x);
	}

	// It's important that a box intersecting with 2 or 4 corners at equal or nearly equal depths doesn't just
	// pick one of the corners; this would lead to unwanted torquing.
	void FindIntersectionBoxGround(Box * box, Ground * ground, std::vector<Intersection> & intersections)
	{
		int id[8];
		float dist[8];
		Vector3 corners[8]={
			Vector3(-box->GetHalfSize()[0], -box->GetHalfSize()[1], -box->GetHalfSize()[2]),
			Vector3(-box->GetHalfSize()[0], -box->GetHalfSize()[1], box->GetHalfSize()[2]),
			Vector3(-box->GetHalfSize()[0], box->GetHalfSize()[1], box->GetHalfSize()[2]),
			Vector3(-box->GetHalfSize()[0], box->GetHalfSize()[1], -box->GetHalfSize()[2]),
			Vector3(box->GetHalfSize()[0], box->GetHalfSize()[1], -box->GetHalfSize()[2]),
			Vector3(box->GetHalfSize()[0], box->GetHalfSize()[1], box->GetHalfSize()[2]),
			Vector3(box->GetHalfSize()[0], -box->GetHalfSize()[1], box->GetHalfSize()[2]),
			Vector3(box->GetHalfSize()[0], -box->GetHalfSize()[1], -box->GetHalfSize()[2]),
		};
		for(int i=0;i<8;i++){
			id[i]=i;
			corners[i] = box->GetTransformation() * corners[i];
			dist[i] = corners[i][1] - ground->GetPosition()[1];
		}
		BubbleSort(id, dist, 8);
		if(dist[0]<0){
			Vector3 contactPoint;
			Vector3 outVector(0,1,0);
			if(dist[3]-dist[0]<error){
				contactPoint = (corners[id[0]]+corners[id[1]]+corners[id[2]]+corners[id[3]])/4;
			}
			else if(dist[1]-dist[0]<error){
				contactPoint = (corners[id[0]]+corners[id[1]])/2;
			}
			else 
				contactPoint = corners[id[0]];
			intersections.push_back(Intersection(box, ground, contactPoint, outVector));
		}
	}

	TEST(FindIntersectionBoxSphere, Intersection)
	{
		Sphere s(2);
		Box b(Vector3(4,5,6));
		std::vector<Intersection> i;

		s.SetPosition(Vector3(7,0,0));
		FindIntersection(&s, &b, i);
		CHECK(i.size() == 0);

		s.SetPosition(Vector3(5,0,0));
		FindIntersection(&s, &b, i);
		CHECK(i.size() == 1);
		CHECK(i.back().bodyA == &b);
		CHECK(i.back().bodyB == &s);
		CHECK(i.back().outVector.positionEquals(Vector3(-1,0,0)));
		CHECK(i.back().contactPoint.positionEquals(Vector3(4,0,0)));
	}

	void FindIntersectionBoxSphere(Box * box, Sphere * sphere, std::vector<Intersection> & intersections)
	{
		// the box is always considered the penetrator.
		// This algorithm is not correct under all circumstances: if the center of the sphere 
		// intersects the box, the resultant outVector will be too short.
		// Since there are several rounds of collision resolution, this is probably okay.
		Vector3 sp = sphere->GetPosition();
		float r = sphere->GetRadius();
		Vector3 p = box->GetClosestPoint(sp);
		// assert(!p.positionEquals(sp)); // uncomment to catch the above error situation
		Vector3 v = p-sp;
		float distSq = v.squaredLength();
		if(distSq < r*r+error)
		{
			float dist = sqrtf(distSq);
			Vector3 vn = v.normalisedCopy();
			intersections.push_back(Intersection(box, sphere, p, vn));
		}
	}

	TEST(FindIntersectionGroundSphere, Intersection)
	{
		Sphere s(2);
		Ground g;
		std::vector<Intersection> i;

		// nonintersecting
		s.SetPosition(Vector3(0,3,0));
		FindIntersection(&s, &g, i);
		CHECK(i.size() == 0);

		// intersecting 1
		s.SetPosition(Vector3(0,1,0));
		FindIntersection(&s, &g, i);
		CHECK(i.size() == 1);
		CHECK(i.back().bodyA == &s);
		CHECK(i.back().bodyB == &g);
		CHECK(i.back().outVector.positionEquals(Vector3(0,1,0)));
		CHECK(i.back().contactPoint.positionEquals(Vector3(0,-1,0)));

		// intersecting 2
		s.SetPosition(Vector3(0,-10,0));
		FindIntersection(&s, &g, i);
		CHECK(i.size() == 2);
		CHECK(i.back().bodyA == &s);
		CHECK(i.back().bodyB == &g);
		CHECK(i.back().outVector.positionEquals(Vector3(0,1,0)));
		CHECK(i.back().contactPoint.positionEquals(Vector3(0,-12,0)));
	}

	// The contact point should be the deepest point of intersection
	void FindIntersectionGroundSphere(Ground * ground, Sphere * sphere, std::vector<Intersection> & intersections)
	{
		Vector3 outVector(0,1,0);
		Vector3 contactPoint = sphere->GetPosition()-sphere->GetRadius()*outVector;
		if(contactPoint[1] < ground->GetPosition()[1]){
			intersections.push_back(Intersection(sphere, ground, contactPoint, outVector));
		}
	}

	TEST(FindIntersectionSphereSphere, Intersection)
	{
		Sphere s1(1);
		Sphere s2(2);
		std::vector<Intersection> i;

		// nonintersecting
		s1.SetPosition(Vector3(4,0,0));
		s2.SetPosition(Vector3(0,0,0));
		FindIntersection(&s2, &s1, i);
		CHECK(i.size() == 0);

		// sort of intersecting
		s1.SetPosition(Vector3(2.5,0,0));
		FindIntersection(&s2, &s1, i);
		CHECK(i.size() == 1);
		CHECK(i.back().bodyA == &s1);
		CHECK(i.back().bodyB == &s2);
		CHECK(i.back().outVector.positionEquals(Vector3(1,0,0)));
		CHECK(i.back().contactPoint.positionEquals(Vector3(1.25,0,0)));
		
		// really intersecting
		s2.SetPosition(Vector3(2.5,0.5,0));
		FindIntersection(&s2, &s1, i);
		CHECK(i.size() == 2);
		CHECK(i.back().outVector.positionEquals(Vector3(0,-1,0)));
		CHECK(i.back().contactPoint.positionEquals(Vector3(2.5,0.25,0)));
	}

	// For sphere intersection, the contact point should be the midpoint between the two sphere centers
	void FindIntersectionSphereSphere(Sphere * sphere, Sphere * sphere2, std::vector<Intersection> & intersections)
	{
		Vector3 outVector = sphere2->GetPosition() - sphere->GetPosition();
		if (outVector.length()<sphere->GetRadius() + sphere2->GetRadius()){	
			Vector3 contactPoint = (sphere2->GetPosition() + sphere->GetPosition()) / 2;
			outVector.normalise();
			intersections.push_back(Intersection(sphere2, sphere, contactPoint, outVector));
		}
	}

	TEST(ComputeOverlap, Intersection)
	{
		FLOATS_EQUAL(0, ComputeOverlap(0,1,2,3));
		FLOATS_EQUAL(0, ComputeOverlap(2,3,0,1));
		FLOATS_EQUAL(1, ComputeOverlap(0,2,1,3));
		FLOATS_EQUAL(1, ComputeOverlap(1,3,0,2));
		FLOATS_EQUAL(2, ComputeOverlap(0,4,1,3));
		FLOATS_EQUAL(2, ComputeOverlap(1,3,0,4));
	}

	float ComputeOverlap(float a, float b, float c, float d)
	{
		if(!(b >= a && d >= c))
		{
			__debugbreak();
		}
		assert(b >= a);
		assert(d >= c);

		if(b < c) return 0;
		if(a > d) return 0;
		return std::min(b,d) - std::max(a,c);
	}

	float ComputeOverlapCentroid(float a, float b, float c, float d)
	{
		assert(b >= a);
		assert(d >= c);

		if(b < c) return 0;
		if(a > d) return 0;

		return (std::min(b,d) + std::max(a,c)) / 2;
	}

}

TEST(TransformPlane, Intersection)
{
	Matrix4 m;
	m.makeTrans(0,4,0);
	Vector4 v = TransformPlane(Vector4(0,1,0,-1), m);
	FLOATS_EQUAL(0, v.x);
	FLOATS_EQUAL(1, v.y);
	FLOATS_EQUAL(0, v.z);
	FLOATS_EQUAL(-5, v.w);

	m.makeTransform(Vector3(0,4,0), Vector3::UNIT_SCALE, Quaternion(float(M_PI_2), Vector3(0,0,1)));
	v = TransformPlane(Vector4(0,1,0,-1), m);
	FLOATS_EQUAL(-1, v.x);
	FLOATS_EQUAL(0, v.y);
	FLOATS_EQUAL(0, v.z);
	FLOATS_EQUAL(-1, v.w);
}

// LOOK transforming a plane is not entirely trivial. Make sure to use this to transform planes,
// rather than simply multiplying the transformation matrix by the plane vector.
Vector4 TransformPlane(Vector4 plane, Matrix4 transform)
{
	Matrix3 R;
	transform.extract3x3Matrix(R);
	Vector3 n(plane.x, plane.y, plane.z);
	Vector3 np = R*n;
	Vector3 translation = transform*Vector3::ZERO;
	float dp = plane.w - (np.dotProduct(translation));
	return Vector4(np.x, np.y, np.z, dp);
}

void BubbleSort (int *u, float *r, int n) {  
	int i= n -1;  
	while ( i> 0) {   
		int pos= 0;   
		for (int j= 0; j< i; j++)  
			if (r[j]> r[j+1]) {  
				pos= j;    
				float tmp = r[j]; r[j] = r[j+1];r[j+1] = tmp;
				int tp = u[j]; u[j] = u[j+1]; u[j+1] = tp;
			}   
			i= pos;
	}   
} 