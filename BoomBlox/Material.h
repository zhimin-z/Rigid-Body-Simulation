#ifndef MATERIAL_H
#define MATERIAL_H

#include <OgreMath.h>

struct Material
{
	Material(float density, float friction, float restitution, Vector3 const& color) :
		density(density), friction(friction), restitution(restitution), color(color)
	{ }

	float density;
	float friction;
	float restitution;
	Vector3 color;

	// Built-in materials, useful for testing stuff
	static Material DIRT;
	static Material BLUE_PLASTIC;
	static Material GREEN_PLASTIC;
	static Material MISSILE_STUFF;
};

#endif