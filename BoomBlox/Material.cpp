#include "Material.h"
#include <limits>

namespace
{
	float inf = std::numeric_limits<float>::infinity();
}

Material Material::DIRT(0, 0.9f, 0.4f, Vector3(0.8f, 0.6f, 0.4f));
Material Material::BLUE_PLASTIC(1, 0.6f, 0.9f, Vector3(0.5f, 0.6f, 0.9f));
Material Material::GREEN_PLASTIC(1, 0, 0.9f, Vector3(0.3f, 0.9f, 0.4f));
Material Material::MISSILE_STUFF(1, 0.6f, 0.9f, Vector3(0.9f, 0.6f, 0.4f));
