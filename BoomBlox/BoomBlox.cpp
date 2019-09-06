#include "Tests.h"
#include <iostream>
#include <string>
#include "GL\freeglut.h"
#include <OgreMath.h>
#include "World.h"
#include "RigidBody.h"
#include "Ground.h"
#include "Box.h"
#include "Sphere.h"
#define NOMINMAX
#include <windows.h>
#include <CommDlg.h>
#include <MMSystem.h>
#include <utility>
#include "Material.h"
#include "WorldLoader.h"

void SetupGraphics();
void RunGame();
void LaunchMissile();
void LoadWorldIntoGame( std::string const& mbfilename );

World g_world;

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	std::cout.sync_with_stdio();

	RunTests();
	SetupGraphics();

	// LOOK if you wanted, you could load a level right here for testing

	RunGame();
}

void display();
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void keyboard(unsigned char key, int x, int y);
void reshape(int width, int height);
void idle();

int g_width = 640;
int g_height = 480;
Vector3 g_cameraPos = Vector3(0,3,30);
// angles in radians
float g_cameraHeading = 0;
float g_cameraPitch = 0;
int g_activeButton = -1;
int g_lastX, g_lastY;
int g_prevTime;
Material* g_missileMaterial = NULL;
int g_stepNum = 0;

const float TRANSLATE_SPEED = 0.1f;
const float ROTATE_SPEED = 0.005f;

void SetupGraphics()
{
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutCreateWindow("Boom Blox!");

	glutDisplayFunc(display);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);

	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset(1,1);
	glClearColor(0.4, 0.5, 1.0, 0.0);
}

void display()
{
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, float(g_width)/g_height, 0.1, 500.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRotatef(g_cameraPitch*180.0f/M_PI, -1, 0, 0);
	glRotatef(g_cameraHeading*180.0f/M_PI, 0, -1, 0);
	glTranslatef(-g_cameraPos.x, -g_cameraPos.y, -g_cameraPos.z);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	float lightPos[] = {-50, 40, 30, 0};
	float ambient[] = {0.4, 0.4, 0.4, 1};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glEnable(GL_COLOR_MATERIAL);

	g_world.Render();

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, g_width, g_height, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glColor3f(1,1,1);
	glBegin(GL_POLYGON);
	glVertex2i(1,1);
	glVertex2i(16,1);
	glVertex2i(16,16);
	glVertex2i(1,16);
	glEnd();
	glColor3f(0,0,0);
	glBegin(GL_LINE_STRIP);
	glVertex2i(5, 3);
	glVertex2i(5, 14);
	glVertex2i(11, 14);
	glEnd();
	glutSwapBuffers();
}

void LoadWorldIntoGame( std::string const& mbfilename )
{
	g_world = LoadWorldFromFile(mbfilename);
	g_missileMaterial = new Material(Material::MISSILE_STUFF);
	g_world.AddMaterial(g_missileMaterial);
}

void PromptToLoad()
{
	OPENFILENAME ofn;
	wchar_t filename[1024];
	memset(filename, 0, sizeof(filename));

	memset(&ofn, 0, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.lpstrFilter = L"World Files\0*.xml\0All Files\0*.*\0\0";
	ofn.lpstrFile = filename;
	ofn.nMaxFile = 1024;
	ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY;

	BOOL result = GetOpenFileName(&ofn);
	if(result)
	{
		char mbfilename[1024];
		wcstombs(mbfilename, filename, 1024);
		LoadWorldIntoGame(mbfilename);

	}
	else
	{
		DWORD err = CommDlgExtendedError();
		err = err;
	}

	g_stepNum = 0;
}
void mouse(int button, int state, int x, int y)
{
	if(state == GLUT_DOWN)
	{
		if(x < 16 && y < 16)
		{
			PromptToLoad();
		}
		else
		{
			g_activeButton = button;
			g_lastX = x;
			g_lastY = y;
		}
	}
	else if(state == GLUT_UP)
	{
		g_activeButton = -1;
	}
}

void motion(int x, int y)
{
	switch(g_activeButton)
	{
	case GLUT_LEFT_BUTTON:
		{
			Vector3 trans((x-g_lastX)*TRANSLATE_SPEED, 0, (y-g_lastY)*TRANSLATE_SPEED);
			trans = Quaternion(g_cameraHeading, Vector3::UNIT_Y) * trans;
			g_cameraPos += trans;
			break;
		}

	case GLUT_RIGHT_BUTTON:
		{
			g_cameraHeading -= (x-g_lastX)*ROTATE_SPEED;
			g_cameraPitch -= (y-g_lastY)*ROTATE_SPEED;
			break;
		}
	default:
		return;
	}
	g_lastX = x;
	g_lastY = y;
}

Vector3 ComputeTrajectory()
{
	// LOOK for better missile control
	const float MISSILE_SPEED = 40;
	float heading = g_cameraHeading;
	float pitch = g_cameraPitch;

	return Vector3(
		-sinf(heading)*cosf(pitch),
		sinf(pitch),
		-cosf(heading)*cosf(pitch)) * MISSILE_SPEED;
}

void LaunchMissile()
{
	Sphere* s = new Sphere(1);
	s->SetMaterial(g_missileMaterial);
	s->SetPosition(g_cameraPos);
	s->SetVelocity(ComputeTrajectory());

	g_world.AddBody(s);
}

void keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
	case ' ':
		LaunchMissile();
		break;
	}
}

void reshape(int width, int height)
{
	g_width = width;
	g_height = height;
	
	glViewport(0, 0, width, height);
}

void idle()
{
	int newTime = timeGetTime();
	float dT = (newTime-g_prevTime) / 1000.0f;
	g_prevTime = newTime;

	dT = std::min(dT, 0.03f);

	g_world.Simulate(dT);
	// LOOK fixed timesteps are much easier to debug
//	g_world.Simulate(0.03f);
	g_stepNum++;
	glutPostRedisplay();
}

void RunGame()
{
	timeBeginPeriod(1);
	g_prevTime = timeGetTime();

	glutMainLoop();
}