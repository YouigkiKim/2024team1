#ifndef WAYPOINT_LOADER_H
#define WAYPOINT_LOADER_H

#include <vector>
#include <string>
#include <utility> // std::pair

class WaypointLoader {
public:
    static std::vector<std::pair<double, double>> loadWaypointsFromFile(const std::string& filepath);
};

#endif // WAYPOINT_LOADER_H
