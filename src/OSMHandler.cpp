#include "OSMHandler.hpp"

#include <osmium/io/reader.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/way.hpp>
#include <unordered_set>

namespace osm_handler
{
    void OSMHandler::printBoundingBox() const
    {
        std::cout << "Bounding box: ("
                  << bbox.bottom_left().lat() << ", " << bbox.bottom_left().lon() << ") to ("
                  << bbox.top_right().lat() << ", " << bbox.top_right().lon() << "), (lat, long)" << std::endl;
    }

    void OSMHandler::node(const osmium::Node &node)
    {
        static bool printed = false;
        if (!printed)
        {
            std::cout << "Node started" << std::endl;
            // printBoundingBox();
            printed = true;
        }

        // Check if the node location is within the bounding box
        if (bbox.contains(node.location()))
        {
            // Temporarily store the node location
            // temp_nodes[node.id()] = {node.location().lat(), node.location().lon()};
            temp_nodes[node.id()] = {node.location().lat(), node.location().lon()};
            // std::cout << "Added node to temp_nodes: " << node.id() << " (" << node.location().lat() << ", " << node.location().lon() << ")" << std::endl;
        }
        else
        {
            if (node.location().lat() < 58 && node.location().lat() > 57 && node.location().lon() < 13 && node.location().lon() > 12)
            {
                std::cout << "Node " << node.id() << " should be added but is not" << std::endl;
            }
            // std::cout << "Node " << node.id() << " is outside the bounding box" << std::endl;
        }
    }

    void OSMHandler::way(const osmium::Way &way)
    {
        /*for (const auto& tag : way.tags()) {
            std::cout << "Tag: " << tag.key() << " = " << tag.value() << std::endl;
        }*/
        // Check if the way is a highway
        // const char *highway = way.tags()["highway"];
        static bool printed = false;
        if (!printed)
        {
            std::cout << "Way started" << std::endl;
            // printBoundingBox();
            printed = true;
        }

        static size_t way_count = 0;
        const char *highway = way.tags()["highway"];
        if (!highway)
        {
            return; // Skip non-highway ways
        }

        // Only include drivable road types
        static const std::unordered_set<std::string> drivable_types = {
            "motorway", "motorway_link",
            "trunk", "trunk_link",
            "primary", "primary_link",
            "secondary", "secondary_link",
            "tertiary", "tertiary_link",
            "residential", "living_street",
            "unclassified", "service",
            "road"
        };

        if (drivable_types.find(highway) == drivable_types.end())
        {
            return; // Skip non-drivable ways (footpaths, cycleways, steps, etc.)
        }
        else
        {
            way_count++;
            // std::cout << "Highway way found: " << way_count << std::endl;
        }

        // way_count++;
        const auto &nodes = way.nodes();
        // std::cout << "Size of nodes: " << nodes.size() << std::endl;
        for (size_t i = 1; i < nodes.size(); i++)
        {
            // Check if the locations are valid (amongst temp_nodes)
            size_t node1 = nodes[i - 1].ref();
            size_t node2 = nodes[i].ref();
            auto it_node1 = temp_nodes.find(node1);
            auto it_node2 = temp_nodes.find(node2);
            if (it_node1 == temp_nodes.end() || it_node2 == temp_nodes.end())
            {
                // std::cout << "Node " << node1 << " or " << node2 << " not found in temp_nodes" << std::endl;
                continue;
            }
            else
            {
                // std::cout << "Node " << node1 << " and " << node2 << " found in temp_nodes" << std::endl;

                // add nodes to the graph:
                graph.addNode(nodes[i - 1].ref(), it_node1->second.first, it_node1->second.second);
                graph.addNode(nodes[i].ref(), it_node2->second.first, it_node2->second.second);
                // std::cout << "Added nodes to the graph: " << nodes[i - 1].ref() << " and " << nodes[i].ref() << std::endl;

                // Add an edge between the nodes
                graph.addEdge(nodes[i - 1].ref(), nodes[i].ref());
                // std::cout << "Edge was added between nodes " << nodes[i - 1].ref() << " and " << nodes[i].ref() << std::endl;
            }
        }
    }
} // namespace osm_handler
