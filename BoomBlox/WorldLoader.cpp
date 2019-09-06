#include "OgreMath.h"
#include "WorldLoader.h"
#include "World.h"
#include "RigidBody.h"
#include "Ground.h"
#include "Box.h"
#include "Sphere.h"
#include "Material.h"
#include <tinyxml.h>
#include <map>

class XMLWorldVisitor : public TiXmlVisitor
{
public:
	XMLWorldVisitor(World* world) : m_world(world), m_curMaterial(NULL), m_curBody(NULL) { }

	bool IsBody(TiXmlNode const* node)
	{
		if(!node->ToElement()) return false;
		if(node->ValueStr() == "ground") return true;
		if(node->ValueStr() == "box") return true;
		if(node->ValueStr() == "sphere") return true;
		return false;
	}

	/// Visit an element.
	virtual bool VisitEnter( const TiXmlElement& element, const TiXmlAttribute* attribute)
	{
		if(element.ValueStr() == "world")
		{
			if(element.Parent() != element.GetDocument()) return false;
			return true;
		}
		else if(element.ValueStr() == "materials")
		{
			if(element.Parent()->ValueStr() != "world") return false;
			return true;
		}
		else if(element.ValueStr() == "material")
		{
			if(element.Parent()->ValueStr() != "materials") return false;

			char const* name;
			double density = 1;
			double friction = 0;
			double restitution = 0;
			double cr = 0;
			double cg = 0;
			double cb = 0;

			name = element.Attribute("name");
			if(!name)
			{
				return false;
			}

			element.Attribute("density", &density);
			element.Attribute("friction", &friction);
			element.Attribute("restitution", &restitution);
			element.Attribute("cr", &cr);
			element.Attribute("cg", &cg);
			element.Attribute("cb", &cb);

			m_curMaterial = new Material(float(density), float(friction), float(restitution), Vector3(float(cr), float(cg), float(cb)));
			m_materials[name] = m_curMaterial;

			return true;
		}
		else if(element.ValueStr() == "bodies")
		{
			if(element.Parent()->ValueStr() != "world") return false;
			return true;
		}
		else if(element.ValueStr() == "ground")
		{
			if(element.Parent()->ValueStr() != "bodies") return false;
			assert(m_curBody == NULL);

			m_curBody = new Ground();
			return true;
		}
		else if(element.ValueStr() == "box")
		{
			if(element.Parent()->ValueStr() != "bodies") return false;
			assert(m_curBody == NULL);

			double hx = 1;
			double hy = 1;
			double hz = 1;
			
			element.Attribute("hx", &hx);
			element.Attribute("hy", &hy);
			element.Attribute("hz", &hz);

			m_curBody = new Box(Vector3(float(hx), float(hy), float(hz)));
			return true;
		}
		else if(element.ValueStr() == "sphere")
		{
			if(element.Parent()->ValueStr() != "bodies") return false;
			assert(m_curBody == NULL);

			double r = 1;

			element.Attribute("r", &r);

			m_curBody = new Sphere(float(r));
			return true;
		}
		else if(element.ValueStr() == "pos")
		{
			if(!IsBody(element.Parent())) return false;
			assert(m_curBody != NULL);
			double x = 0;
			double y = 0;
			double z = 0;

			element.Attribute("x", &x);
			element.Attribute("y", &y);
			element.Attribute("z", &z);

			m_curBody->SetPosition(Vector3(float(x), float(y), float(z)));
			return true;
		}
		else if(element.ValueStr() == "vel")
		{
			if(!IsBody(element.Parent())) return false;
			assert(m_curBody != NULL);
			double x = 0;
			double y = 0;
			double z = 0;

			element.Attribute("x", &x);
			element.Attribute("y", &y);
			element.Attribute("z", &z);

			m_curBody->SetVelocity(Vector3(float(x), float(y), float(z)));
			return true;
		}
		else if(element.ValueStr() == "ori")
		{
			if(!IsBody(element.Parent())) return false;
			assert(m_curBody != NULL);
			double theta = 0;
			double x = 0;
			double y = 0;
			double z = 0;

			element.Attribute("theta", &theta);
			element.Attribute("x", &x);
			element.Attribute("y", &y);
			element.Attribute("z", &z);

			m_curBody->SetOrientation(Quaternion(float(theta), Vector3(float(x), float(y), float(z))));
			return true;
		}
		else if(element.ValueStr() == "avel")
		{
			if(!IsBody(element.Parent())) return false;
			assert(m_curBody != NULL);
			double x = 0;
			double y = 0;
			double z = 0;

			element.Attribute("x", &x);
			element.Attribute("y", &y);
			element.Attribute("z", &z);

			m_curBody->SetAngularVelocity(Vector3(float(x), float(y), float(z)));
			return true;
		}
		else if(element.ValueStr() == "bodymaterial")
		{
			if(!IsBody(element.Parent())) return false;
			assert(m_curBody != NULL);

			const char* name = element.Attribute("name");
			Material* material = m_materials[name];
			if(material != NULL)
			{
				m_curBody->SetMaterial(material);
			}
			return true;
		}
		else
		{
			return false;
		}

		assert(false); // we should never get here
		return false;
	}
	/// Visit an element.
	virtual bool VisitExit( const TiXmlElement& element)
	{
		if(element.ValueStr() == "world")
		{
			return true;
		}
		else if(element.ValueStr() == "materials")
		{
			return true;
		}
		else if(element.ValueStr() == "material")
		{
			m_world->AddMaterial(m_curMaterial);
			m_curMaterial = NULL;
			return true;
		}
		else if(element.ValueStr() == "bodies")
		{
			return true;
		}
		else if(element.ValueStr() == "ground")
		{
			m_world->AddBody(m_curBody);
			m_curBody = NULL;
			return true;
		}
		else if(element.ValueStr() == "box")
		{
			m_world->AddBody(m_curBody);
			m_curBody = NULL;
			return true;
		}
		else if(element.ValueStr() == "sphere")
		{
			m_world->AddBody(m_curBody);
			m_curBody = NULL;
			return true;
		}
		else if(element.ValueStr() == "pos")
		{
			return true;
		}
		else if(element.ValueStr() == "vel")
		{
			return true;
		}
		else if(element.ValueStr() == "ori")
		{
			return true;
		}
		else if(element.ValueStr() == "avel")
		{
			return true;
		}
		else if(element.ValueStr() == "bodymaterial")
		{
			return true;
		}
		else
		{
			return false;
		}
	}

private:
	World* m_world;
	Material* m_curMaterial;
	RigidBody* m_curBody;
	std::map<std::string, Material*> m_materials;
};
World LoadWorldFromFile(std::string const& filename)
{
	TiXmlDocument doc(filename);
	bool success = doc.LoadFile();
	if(!success)
	{
		return World();
	}

	World world;
	XMLWorldVisitor v(&world);
	doc.Accept(&v);

	return world;
}