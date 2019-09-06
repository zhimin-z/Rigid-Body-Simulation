#include "Tests.h"

#include <iostream>
#include <string>

#include "Sphere.h"
#include "Box.h"

#include <TestHarness.h>

void RunTests()
{
	TestResult tr;
	TestRegistry::runAllTests(tr);
}