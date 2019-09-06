#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <string>

// LOOK an easy way to benchmark your broadphase collision test. Results print
// to the console. Call this instead of RunGame. Don't call it at all in the 
// code you submit.

void RunBenchmark(std::string const& name, int frames=100);

#endif