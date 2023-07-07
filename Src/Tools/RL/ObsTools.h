// Tools for getting observations from the environment
#ifndef ObsTools_h
#define ObsTools_h

#include "Tools/RLConfig.h"
#include <vector>

std::vector<float> get_relative_observation(std::vector<float> agent_loc, std::vector<float> object_loc);
bool isFacingPoint(float x, float y, float angle);

#endif

