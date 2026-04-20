
#include "graph.hpp"
#include "utils.hpp"
#include <iostream>
#include <cmath>

int main()
{
    graph::Graph g;
    try
    {
        std::string file_path = "../../sweden-latest.osm.pbf";
        std::cout << "OSM file: " << file_path << std::endl;

        // Set the source OSM file path
        g.setFilePath(file_path);

        // Load data at startup for fast searches and route calculations.
        // 6 GB is treated as a target (monitoring), not a hard cap.
        g.preloadDataForFastRouting(6.0f);

        std::cout << "Server starting (graph preloaded at startup)..." << std::endl;

        // Start the WebSocket server
        g.startServer();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Example: Find shortest path between two nodes:
    /*t current = 123456, target = 789012; // Replace with real OSM node IDs
    g.dijkstra(current, target);*/
}
