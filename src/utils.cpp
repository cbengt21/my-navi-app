
#include <osmium/io/reader.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/box.hpp>
#include <osmium/io/pbf_input.hpp>

#include "utils.hpp"
#include "OSMHandler.hpp"

namespace utils
{
    void printtoconsole(std::string message)
    {
        std::cout << "Debug message: " << message << std::endl;
    }

    // Haversine formula to calculate distance between two GPS coordinates
    float haversine(float lat1, float lon1, float lat2, float lon2)
    {
        constexpr float R = 6371e3;
        float dLat = (lat2 - lat1) * M_PI / 180.0;
        float dLon = (lon2 - lon1) * M_PI / 180.0;
        float a = sin(dLat / 2) * sin(dLat / 2) +
                  cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
                      sin(dLon / 2) * sin(dLon / 2);
        return R * (2 * atan2(sqrt(a), sqrt(1 - a)));
    }

    void loadTile(graph::Graph &g, float min_lon, float min_lat, float max_lon, float max_lat, std::string file_path)
    {
        auto start = high_resolution_clock::now();
        osmium::Box bbox(osmium::Location(min_lon, min_lat), osmium::Location(max_lon, max_lat));
        // std::cout << "Bounding box: (" << min_lon << ", " << min_lat << ") to (" << max_lon << ", " << max_lat << "), (long, lat)" << std::endl;
        osm_handler::OSMHandler handler(g, bbox);
        // handler.printBoundingBox();
        osmium::io::Reader reader(file_path, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way, osmium::io::read_meta::no);
        // OSMHandler handler(g, bbox);

        // Process ways and nodes together
        osmium::apply(reader, handler);
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(stop - start);
        std::cout << "Time taken to load tile: " << duration.count() << " milliseconds" << std::endl;
        g.printNrOfNodes();
        reader.close();
        // std::cout << "Size of temp_nodes: " << handler.temp_nodes.size() << std::endl;
        handler.temp_nodes.clear(); // Clear the temporary nodes
        std::unordered_map<int64_t, std::pair<float, float>>().swap(handler.temp_nodes);
    }
} // namespace utils
