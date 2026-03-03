#ifndef OSMHANDLER_HPP_
#define OSMHANDLER_HPP_

#include <osmium/handler.hpp>
#include <osmium/osm/box.hpp>
#include <unordered_map>
#include <unordered_set>
#include "graph.hpp"

namespace osm_handler
{
    // Pass 1: Collects node IDs referenced by drivable highway ways
    class WayNodeCollector : public osmium::handler::Handler
    {
        std::unordered_set<int64_t> &needed_nodes;

    public:
        WayNodeCollector(std::unordered_set<int64_t> &nodes) : needed_nodes(nodes) {}

        void way(const osmium::Way &way);
    };

    // Pass 2: OSM Data Loader, to Parse OSM Data and Build Graph:
    class OSMHandler : public osmium::handler::Handler
    {
        graph::Graph &graph;
        osmium::Box bbox;
        const std::unordered_set<int64_t> &needed_nodes; // Only store these node IDs
        size_t way_count = 0;

    public:
        std::unordered_map<int64_t, std::pair<float, float>> temp_nodes; // Temporarily store node locations
        // Constructor:
        OSMHandler(graph::Graph &g, const osmium::Box &box, const std::unordered_set<int64_t> &needed)
            : graph(g), bbox(box), needed_nodes(needed) { printBoundingBox(); }

        void printBoundingBox() const;

        void node(const osmium::Node &node);

        void way(const osmium::Way &way);
    };
} // namespace osm_handler
#endif // OSMHANDLER_HPP_
