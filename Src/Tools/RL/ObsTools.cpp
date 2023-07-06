#include "ObsTools.h"

#include <cmath>
#include <vector>


// def get_relative_observation(self, agent_loc, object_loc):
std::vector<float> get_relative_observation(std::vector<float> agent_loc, std::vector<float> object_loc) {
    // Get relative position of object to agent, returns x, y, angle
    // Agent loc is x, y, angle
    // Object loc is x, y

    // Get relative position of object to agent
    float x = object_loc[0] - agent_loc[0];
    float y = object_loc[1] - agent_loc[1];

    float angle = std::atan2(y, x) - agent_loc[2];
    // angle = -1 * (angle - 180);

    // print angle
    // std::cout << "Angle: " << angle << std::endl;

    // Rotate x, y by -agent angle
    float xprime = x * std::cos(-agent_loc[2]) - y * std::sin(-agent_loc[2]);
    float yprime = x * std::sin(-agent_loc[2]) + y * std::cos(-agent_loc[2]);

    return {xprime/5200, yprime/3500, std::sin(angle), std::cos(angle)};
}
