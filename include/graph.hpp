#ifndef GRAPH_HPP_
#define GRAPH_HPP_
#include <unordered_map>
#include <vector>
#include <crow.h>
#include <chrono>
using namespace std::chrono;

namespace graph
{
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
        std::unordered_map<int64_t, Node> nodes;                // Node ID -> (lat, lon)

    public:
        void printNrOfNodes()
        {
            std::cout << "Number of nodes: " << nodes.size() << std::endl;
        }

        // addNode and addEdge methods to build the graph:
        void addNode(int64_t id, float lat, float lon);
        void addEdge(int64_t u, int64_t v);

        // clear the graph if necessary
        void clearGraph();

        // Dijkstra's algorithm:
        std::vector<int64_t> dijkstra(int64_t current, int64_t target);

        // A* algorithm:
        std::vector<int64_t> a_star(int64_t start, int64_t target);

        crow::json::wvalue getPathAsJSON(int64_t current, int64_t target);

        ~Graph()
        {
            // Stop the Crow server
            app.stop();
        }

        // Method to start the Crow server
        void startServer();

        int64_t findNearestNode(float lat, float lon);
    };
} // namespace graph
#endif // GRAPH_HPP_
