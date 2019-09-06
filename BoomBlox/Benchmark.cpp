#include "Benchmark.h"
#include "Intersection.h"
#include "World.h"
#include "WorldLoader.h"
#include <vector>
#include <windows.h>

void LoadWorldIntoGame( std::string const& mbfilename );
extern World g_world;

void RunBenchmark(std::string const& name, int frames)
{
	timeBeginPeriod(1);

	LoadWorldIntoGame(name);
	std::vector<Intersection> intersections;

	long startTime = timeGetTime();
	int numIntersections = 0;
	for(int i=0; i<frames; i++)
	{
		g_world.FindIntersections(intersections);
		numIntersections += int(intersections.size());
		intersections.clear();
		g_world.AdvancePositions(0.01f);
	}
	long endTime  = timeGetTime();

	std::cout << name + ": " << endTime-startTime << " ticks, " << numIntersections << " intersections.\n";
	std::cout.flush();

	timeEndPeriod(1);
}