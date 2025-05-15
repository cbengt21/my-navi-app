
#include "graph.hpp"
#include "utils.hpp"

#include <queue>
#include <unordered_set>
#include <nlohmann/json.hpp>

namespace graph
{
    void Graph::addNode(int64_t id, float lat, float lon)
    {
        // Only add the node if it doesn't already exist
        if (nodes.find(id) == nodes.end())
        {
            nodes[id] = std::make_shared<Node>(Node{lat, lon});
            // std::cout << "Node added: " << id << " (" << lat << ", " << lon << ")" << std::endl;
        }
    }
    void Graph::addEdge(int64_t u, int64_t v)
    {
        if (nodes.count(u) && nodes.count(v))
        {
            float distance = utils::haversine(nodes[u]->lat, nodes[u]->lon, nodes[v]->lat, nodes[v]->lon);
            adjList[u].push_back({v, distance});
            adjList[v].push_back({u, distance});
        }
    }

    // clear the graph if necessary
    void Graph::clearGraph()
    {
        nodes.clear();
        adjList.clear();
    }

    // Dijkstra's algorithm:
    std::vector<int64_t> Graph::dijkstra(int64_t current, int64_t target)
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
    std::vector<int64_t> Graph::a_star(int64_t start, int64_t target)
    {
        std::cout << "Running A* algo" << std::endl;

        // Cache target node coordinates to avoid repeated map lookups
        const float target_lat = nodes[target]->lat;
        const float target_lon = nodes[target]->lon;

        auto heuristic = [&, target_lat, target_lon](int64_t from)
        {
            const auto &n = nodes[from];
            return utils::haversine(n->lat, n->lon, target_lat, target_lon);
        };

        using PQEntry = std::pair<float, int64_t>; // (f_score, node)
        std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> pq;

        std::unordered_map<int64_t, float> g_score;    // Distance from start (lazy: missing = INFINITY)
        std::unordered_map<int64_t, int64_t> previous; // Path reconstruction
        std::unordered_set<int64_t> visited;            // Skip already-settled nodes

        g_score[start] = 0;
        pq.push({heuristic(start), start});

        while (!pq.empty())
        {
            auto [f, current] = pq.top();
            pq.pop();

            if (current == target)
                break;

            // Skip stale entries
            if (visited.count(current))
                continue;
            visited.insert(current);

            float current_g = g_score[current];

            for (const auto &[neighbor, weight] : adjList[current])
            {
                if (visited.count(neighbor))
                    continue;

                float tentative_g = current_g + weight;
                auto it = g_score.find(neighbor);
                if (it == g_score.end() || tentative_g < it->second)
                {
                    g_score[neighbor] = tentative_g;
                    pq.push({tentative_g + heuristic(neighbor), neighbor});
                    previous[neighbor] = current;
                }
            }
        }

        auto it = g_score.find(target);
        if (it == g_score.end())
            return {}; // No path found

        // Reconstruct path:
        std::vector<int64_t> path;
        for (int64_t at = target; at != start; at = previous[at])
        {
            path.push_back(at);
        }

        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return path;
    }

    crow::json::wvalue Graph::getPathAsJSON(int64_t current, int64_t target)
    {
        auto path = a_star(current, target);
        std::cout << "A* algo completed, path has " << path.size() << " nodes" << std::endl;
        crow::json::wvalue response;
        for (size_t i = 0; i < path.size(); i++)
        {
            const auto &node = nodes[path[i]];
            if (node != nullptr)
            {
                response["path"][i]["id"] = path[i];
                response["path"][i]["lat"] = node->lat;
                response["path"][i]["lon"] = node->lon;
            }
            else
            {
                std::cerr << "Error: Node " << path[i] << " is null or not found" << std::endl;
            }
        }
        std::cout << "Path JSON generated" << std::endl;
        return response;
    }

    // Method to start the Crow server
    void Graph::startServer()
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

    int64_t Graph::findNearestNode(float lat, float lon)
    {
        int64_t nearest_node = -1;
        float min_distance = std::numeric_limits<float>::max();

        for (const auto &[id, node] : nodes)
        {
            if (node) // Ensure node is not null
            {
                float distance = utils::haversine(lat, lon, node->lat, node->lon);
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
} // namespace graph
