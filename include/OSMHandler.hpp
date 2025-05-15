#ifndef OSMHANDLER_HPP_
#define OSMHANDLER_HPP_

#include <osmium/handler.hpp>
#include <osmium/osm/box.hpp>
#include <unordered_map>
#include "graph.hpp"

namespace osm_handler
{
    // OSM Data Loader, to Parse OSM Data and Build Graph:
    class OSMHandler : public osmium::handler::Handler
    {
        graph::Graph &graph;
        osmium::Box bbox;
        size_t way_count = 0;

    public:
        std::unordered_map<int64_t, std::pair<float, float>> temp_nodes; // Temporarily store node locations
        // Constructor:
        OSMHandler(graph::Graph &g, const osmium::Box &box) : graph(g), bbox(box) { printBoundingBox(); }

        void printBoundingBox() const;

        void node(const osmium::Node &node);

        void way(const osmium::Way &way);
    };
} // namespace osm_handler
#endif // OSMHANDLER_HPP_
