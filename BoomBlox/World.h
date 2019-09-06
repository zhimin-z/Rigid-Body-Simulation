#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <string>
#include <TestHarness.h>
#include "Intersection.h"

class RigidBody;
struct Material;

class World
{
public:
	World();
	~World();
	World(World const& other);
	World &operator=(World const& other);

	// Add a material for use by the simulation.
	// he World takes ownership of the material, and is responsible for its deletion.
	void AddMaterial(Material* material);
	int GetNumMaterials() const;
	Material const* GetMaterial(int material) const;

	// Add a body to the simulation.
	// The World takes ownership of the body, and is responsible for its deletion.
	// This should only be done after the body's material has already been added.
	void AddBody(RigidBody* body);
	
	// Get the number of bodies already added to the simulation.
	int GetNumBodies() const;
	RigidBody const* GetBody(int body) const;
	RigidBody* GetBody(int body);

	int FindBody(RigidBody* body) const;

	void RemoveBody(int body);

	// Remove all bodies and materials, freeing them.
	void Clear();
	void Render() const;

	// Single simulation step.
	void Simulate(float dT);

	//Sweep & Prune - make it run faster!
    bool World::SweepnPrune(int i, int j);

private:
	void FillFrom(World const& other);

	void ResolveCollisions(float dT);
	void AdvanceVelocities(float dT);
	void ResolveContacts(float dT);
	void AdvancePositions(float dT);

	static void ResolveIntersection(Intersection &i, float epsilon, bool immediate = false);

	void SaveState();
	void RestoreState();

	void FindIntersections(std::vector<Intersection> & intersections);

	ALLOW_TEST_ACCESS(AdvancePositions, World);
	ALLOW_TEST_ACCESS(AdvanceVelocities, World);
	ALLOW_TEST_ACCESS(FindIntersections, World);
	ALLOW_TEST_ACCESS(ResolveIntersection, World);


	std::vector<Material*> m_materials;
	std::vector<RigidBody*> m_bodies;

	friend void RunBenchmark(std::string const& name, int frames);
};

#endif