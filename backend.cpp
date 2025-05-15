// #define CROW_USE_BOOST  // Ensure Crow and your project both use Boost.Asio
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <osmium/io/pbf_input.hpp>
#include <crow.h>
#include <osmium/io/reader.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/osm/box.hpp>
#include <cmath>
#include <set>
#include <signal.h>
#include <nlohmann/json.hpp>
#include <iomanip> // for std::setprecision
#include <chrono>
using namespace std::chrono;

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

// Graph representation
struct Node
{
    float lat, lon;
};
struct Edge
{
    int64_t to;
    float weight;
};

// Graph class, with Dijkstra's algorithm and WebSocket server
class Graph
{
private:
    std::unordered_map<int64_t, std::vector<Edge>> adjList; // Adjacency list
    crow::SimpleApp app;                                    // Store the Crow app instance

public:
    std::unordered_map<int64_t, std::shared_ptr<Node>> nodes; // Node ID -> (lat, lon)

    // addNode and addEdge methods to build the graph:
    void addNode(int64_t id, float lat, float lon)
    {
        // Only add the node if it doesn't already exist
        if (nodes.find(id) == nodes.end())
        {
            nodes[id] = std::make_shared<Node>(Node{lat, lon});
            // std::cout << "Node added: " << id << " (" << lat << ", " << lon << ")" << std::endl;
        }
    }
    void addEdge(int64_t u, int64_t v)
    {
        if (nodes.count(u) && nodes.count(v))
        {
            float distance = haversine(nodes[u]->lat, nodes[u]->lon, nodes[v]->lat, nodes[v]->lon);
            adjList[u].push_back({v, distance});
            adjList[v].push_back({u, distance});
        }
    }

    // clear the graph if necessary
    void clearGraph()
    {
        nodes.clear();
        adjList.clear();
    }

    // Dijkstra's algorithm:
    std::vector<int64_t> dijkstra(int64_t current, int64_t target)
    {
        std::cout << "Running Dijkstra algo" << std::endl;
        std::priority_queue<std::pair<float, int64_t>, std::vector<std::pair<float, int64_t>>, std::greater<>> pq;
        std::unordered_map<int64_t, float> distances;
        std::unordered_map<int64_t, int64_t> previous;

        for (auto &[id, _] : nodes)
            distances[id] = INFINITY;
        distances[current] = 0;
        pq.push({0, current});

        while (!pq.empty())
        {
            auto [currentDist, current] = pq.top();
            pq.pop();
            if (current == target)
                break;

            for (auto &[next, weight] : adjList[current])
            {
                float newDist = currentDist + weight;
                if (newDist < distances[next])
                {
                    distances[next] = newDist;
                    pq.push({newDist, next});
                    previous[next] = current;
                }
            }
        }

        if (distances[target] == INFINITY)
            return {}; // No path found

        // Reconstruct path:
        std::vector<int64_t> path;
        for (int64_t at = target; at != current; at = previous[at])
        {
            if (nodes.find(at) == nodes.end() || nodes[at] == nullptr)
            {
                std::cerr << "Error: Node " << at << "is null or not found" << std::endl;
                return {};
            }
            path.push_back(at);
        }

        path.push_back(current);
        reverse(path.begin(), path.end());
        return path;
    }

    // A* algorithm:
    std::vector<int64_t> a_star(int64_t start, int64_t target)
    {
        std::cout << "Running A* algo" << std::endl;
        auto heuristic = [this](int64_t from, int64_t to)
        {
            return haversine(nodes[from]->lat, nodes[from]->lon,
                             nodes[to]->lat, nodes[to]->lon);
        };

        using PQEntry = std::pair<float, int64_t>; // (f_score, node)
        std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> pq;

        std::unordered_map<int64_t, float> g_score;    // Distance from start
        std::unordered_map<int64_t, float> f_score;    // g + h
        std::unordered_map<int64_t, int64_t> previous; // Path reconstruction

        for (const auto &[id, _] : nodes)
        {
            g_score[id] = INFINITY;
            f_score[id] = INFINITY;
        }

        g_score[start] = 0;
        f_score[start] = heuristic(start, target);
        pq.push({f_score[start], start});

        while (!pq.empty())
        {
            auto [_, current] = pq.top();
            pq.pop();

            if (current == target)
                break;

            for (const auto &[neighbor, weight] : adjList[current])
            {
                float tentative_g = g_score[current] + weight;
                if (tentative_g < g_score[neighbor])
                {
                    g_score[neighbor] = tentative_g;
                    f_score[neighbor] = tentative_g + heuristic(neighbor, target);
                    pq.push({f_score[neighbor], neighbor});
                    previous[neighbor] = current;
                }
            }
        }

        if (g_score[target] == INFINITY)
            return {}; // No path found

        // Reconstruct path:
        std::vector<int64_t> path;
        for (int64_t at = target; at != start; at = previous[at])
        {
            if (nodes.find(at) == nodes.end() || nodes[at] == nullptr)
            {
                std::cerr << "Error: Node " << at << " is null or not found" << std::endl;
                return {};
            }
            path.push_back(at);
        }

        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    crow::json::wvalue getPathAsJSON(int64_t current, int64_t target)
    {
        auto path = dijkstra(current, target);
        // auto path = a_star(current, target);
        std::cout << "Dijkstra algo completed" << std::endl;
        crow::json::wvalue response;
        for (size_t i = 0; i < path.size(); i++)
        {
            std::cout << "Processing node " << path[i] << std::endl;
            if (nodes.find(path[i]) != nodes.end() && nodes[path[i]] != nullptr)
            {
                response["path"][i]["id"] = path[i];
                response["path"][i]["lat"] = nodes[path[i]]->lat;
                response["path"][i]["lon"] = nodes[path[i]]->lon;
            }
            else
            {
                std::cerr << "Error: Node " << path[i] << "is null or not found" << std::endl;
            }
        }
        std::cout << "Path JSON generated" << std::endl;
        return response;
    }

    ~Graph()
    {
        // Stop the Crow server
        app.stop();
    }

    // Method to start the Crow server
    void startServer()
    {
        CROW_WEBSOCKET_ROUTE(app, "/ws")
            .onopen([](crow::websocket::connection &conn)
                    { std::cout << "WebSocket connected\n"; })
            .onmessage([this](crow::websocket::connection &conn, const std::string &msg, bool is_binary)
                       {
                        std::cout << "Received: " << msg << std::endl;

                        try
                        {
                            std::cout << std::fixed << std::setprecision(8); // Or however many digits you need
                            // Parse the JSON input
                            auto json_msg = nlohmann::json::parse(msg);
                            std::cout << "Parsed JSON successfully" << std::endl;

                            // Extract the start and end coordinates
                            float start_lat = json_msg["start"]["lat"];
                            float start_lon = json_msg["start"]["lon"];
                            float end_lat = json_msg["end"]["lat"];
                            float end_lon = json_msg["end"]["lon"];
                            std::cout << "Extracted coordinates successfully " << start_lat << " " << start_lon << " " << end_lat << " " << end_lon << std::endl;

                            // Find the nearest nodes to the start and end coordinates
                            int64_t start_node = findNearestNode(start_lat, start_lon);
                            int64_t end_node = findNearestNode(end_lat, end_lon);
                            std::cout << "Found nearest nodes successfully" << std::endl;

                            auto start = high_resolution_clock::now();
                            // Get the path as JSON
                            crow::json::wvalue response = getPathAsJSON(start_node, end_node);
                            std::cout << "Generated path JSON successfully" << std::endl;
                            auto stop = high_resolution_clock::now();
                            auto duration = duration_cast<milliseconds>(stop - start);
                            std::cout << "Time taken to find route: " << duration.count() << " milliseconds" << std::endl;

                            conn.send_text(response.dump());
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
                        } })
            .onclose([](crow::websocket::connection &conn, const std::string &reason, uint16_t code)
                     { std::cout << "WebSocket closed. Reason: " << reason << ", Code: " << code << std::endl; });

        std::cout << "Server running on ws://localhost:18080/ws" << std::endl;
        app.port(18080).multithreaded().run();
    }

    int64_t findNearestNode(float lat, float lon)
    {
        int64_t nearest_node = -1;
        float min_distance = std::numeric_limits<float>::max();

        for (const auto &[id, node] : nodes)
        {
            if (node) // Ensure node is not null
            {
                float distance = haversine(lat, lon, node->lat, node->lon);
                // std::cout << "Node " << id << ": distance = " << distance << std::endl;
                if (distance < min_distance)
                {
                    min_distance = distance;
                    nearest_node = id;
                }
            }
        }

        if (nearest_node == -1)
        {
            std::cerr << "Error: Nearest node not found for coordinates (" << lat << ", " << lon << ")" << std::endl;
        }
        else if (nodes[nearest_node])
        {
            std::cout << "Nearest node: " << nearest_node << " with distance " << min_distance << std::endl;
            std::cout << "Nearest node coordinates: lat = "
                      << std::fixed << std::setprecision(8) << nodes[nearest_node]->lat
                      << ", lon = " << nodes[nearest_node]->lon << std::endl;
        }

        return nearest_node;
    }
};

// OSM Data Loader, to Parse OSM Data and Build Graph:
class OSMHandler : public osmium::handler::Handler
{
    Graph &graph;
    osmium::Box bbox;
    size_t way_count = 0;

public:
    std::unordered_map<int64_t, std::pair<float, float>> temp_nodes; // Temporarily store node locations
    // Constructor:
    OSMHandler(Graph &g, const osmium::Box &box) : graph(g), bbox(box) { printBoundingBox(); }

    void printBoundingBox() const
    {
        std::cout << "Bounding box: ("
                  << bbox.bottom_left().lat() << ", " << bbox.bottom_left().lon() << ") to ("
                  << bbox.top_right().lat() << ", " << bbox.top_right().lon() << "), (lat, long)" << std::endl;
    }

    void node(const osmium::Node &node)
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

    void way(const osmium::Way &way)
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
        const bool highway = way.tags().has_key("highway"); // if not working consider has_tag()
        if (!highway)
        {
            return; // Skip non-highway ways
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
};

void loadTile(Graph &g, float min_lon, float min_lat, float max_lon, float max_lat, std::string file_path)
{
    auto start = high_resolution_clock::now();
    osmium::Box bbox(osmium::Location(min_lon, min_lat), osmium::Location(max_lon, max_lat));
    // std::cout << "Bounding box: (" << min_lon << ", " << min_lat << ") to (" << max_lon << ", " << max_lat << "), (long, lat)" << std::endl;
    OSMHandler handler(g, bbox);
    // handler.printBoundingBox();
    osmium::io::Reader reader(file_path, osmium::osm_entity_bits::node | osmium::osm_entity_bits::way, osmium::io::read_meta::no);
    // OSMHandler handler(g, bbox);

    // Process ways and nodes together
    osmium::apply(reader, handler);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    std::cout << "Time taken to load tile: " << duration.count() << " milliseconds" << std::endl;
    std::cout << "Total nodes loaded: " << g.nodes.size() << std::endl;
    reader.close();
    //std::cout << "Size of temp_nodes: " << handler.temp_nodes.size() << std::endl;
    handler.temp_nodes.clear(); // Clear the temporary nodes
    std::unordered_map<int64_t, std::pair<float, float>>().swap(handler.temp_nodes);
}

int main()
{
    Graph g;
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
        loadTile(g, min_lon, min_lat, max_lon, max_lat, file_path);
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
