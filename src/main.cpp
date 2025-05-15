
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
        std::cout << "Attempting to open file: " << file_path << std::endl;

        // Define the user's location (example coordinates), Alexanderplatz, Berlin:
        // float user_lon = 13.413215;
        // float user_lat = 52.521919;
        // Define the user's location (example coordinates), kungsbacka, Sweden:
        // constexpr float user_lon = 12.0061952;
        // constexpr float user_lat = 57.6847872;

        // Define the tile size (1x1 degree)
        // constexpr float tile_size = 3.0;

        // Calculate the bounding box for the tile containing the user's location
        /*constexpr float min_lat = user_lat - tile_size / 2.0;
        constexpr float max_lat = user_lat + tile_size / 2.0;
        constexpr float min_lon = user_lon - tile_size / 2.0;
        constexpr float max_lon = user_lon + tile_size / 2.0;*/

        // Use the whole map:
        constexpr float min_lat = -90.0;
        constexpr float max_lat = 90.0;
        constexpr float min_lon = -180.0;
        constexpr float max_lon = 180.0;

        // Load and process the tile
        utils::loadTile(g, min_lon, min_lat, max_lon, max_lat, file_path);
        std::cout << "Tile loaded successfully!" << std::endl;

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
