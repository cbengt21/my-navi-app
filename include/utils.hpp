#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <iostream>

#include "graph.hpp"

namespace utils
{
    void printtoconsole(std::string message);

    // Haversine formula to calculate distance between two GPS coordinates
    float haversine(float lat1, float lon1, float lat2, float lon2);

    void loadTile(graph::Graph &g, float min_lon, float min_lat, float max_lon, float max_lat, std::string file_path);
}
#endif // UTILS_HPP_
