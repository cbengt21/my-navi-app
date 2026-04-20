#ifndef GRAPH_HPP_
#define GRAPH_HPP_
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <mutex>
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

    struct RouteResult
    {
        std::vector<int64_t> path;
        float travel_time_seconds = 0.0f;
        std::unordered_set<int64_t> visited; // Nodes explored (populated on failure)
    };

    // Loaded bounding box tracker
    struct LoadedArea
    {
        float min_lat = 0, max_lat = 0, min_lon = 0, max_lon = 0;
        bool valid = false;

        bool contains(float lat, float lon, float margin = 0.3f) const
        {
            return valid &&
                   lat >= (min_lat + margin) && lat <= (max_lat - margin) &&
                   lon >= (min_lon + margin) && lon <= (max_lon - margin);
        }

        float area() const
        {
            return (max_lat - min_lat) * (max_lon - min_lon);
        }
    };

    // Graph class, with A* algorithm and WebSocket server
    class Graph
    {
    private:
        std::unordered_map<int64_t, std::vector<Edge>> adjList;
        std::unordered_map<int64_t, std::vector<Edge>> reverseAdj; // Reverse adjacency (for bidirectional A* and reachability)
        std::unordered_map<int64_t, uint16_t> incoming_count;      // Number of incoming edges per node
        crow::SimpleApp app;
        std::unordered_map<int64_t, Node> nodes;
        std::string osm_file_path;
        LoadedArea loaded_area;
        bool startup_preloaded = false;
        std::mutex graph_mutex;

        // Spatial grid for fast nearest-node lookup
        static constexpr float GRID_CELL_SIZE = 0.01f; // ~1km cells
        struct GridKey
        {
            int lat_idx, lon_idx;
            bool operator==(const GridKey &o) const { return lat_idx == o.lat_idx && lon_idx == o.lon_idx; }
        };
        struct GridKeyHash
        {
            size_t operator()(const GridKey &k) const
            {
                return std::hash<int64_t>()(((int64_t)k.lat_idx << 32) | (uint32_t)k.lon_idx);
            }
        };
        std::unordered_map<GridKey, std::vector<int64_t>, GridKeyHash> spatial_grid;

        GridKey toGridKey(float lat, float lon) const
        {
            return {(int)std::floor(lat / GRID_CELL_SIZE), (int)std::floor(lon / GRID_CELL_SIZE)};
        }
        void buildSpatialGrid();
        int64_t findNearestInGrid(float lat, float lon);
        size_t reverseReachCount(int64_t node, size_t max_depth = 50, size_t max_nodes = 500);
        // Cached reverse-reachability results
        std::unordered_set<int64_t> well_connected_nodes; // Nodes confirmed reachable from/to main network
        void buildWellConnectedSet();

        // Union-Find (Disjoint Set Union) for connected components
        std::unordered_map<int64_t, int64_t> uf_parent;
        std::unordered_map<int64_t, int64_t> uf_rank;
        int64_t largest_component_root = -1;
        int64_t ufFind(int64_t x);
        void ufUnion(int64_t a, int64_t b);
        void buildComponents();
        int64_t findNearestInComponent(float lat, float lon, int64_t component_root);

    public:
        void printNrOfNodes()
        {
            std::cout << "Number of nodes: " << nodes.size() << std::endl;
        }

        void setFilePath(const std::string &path) { osm_file_path = path; }

        // Load graph data once at startup for fast routing/searches.
        // target_memory_gb is a soft target used for logging/monitoring.
        void preloadDataForFastRouting(float target_memory_gb = 6.0f);

        // Ensure the area covering start and end is loaded
        void ensureAreaLoaded(float start_lat, float start_lon, float end_lat, float end_lon, float margin_scale = 1.0f, bool force = false);

        void addNode(int64_t id, float lat, float lon);
        void addEdge(int64_t u, int64_t v, float speed_kmh = 50.0f, bool oneway = false, bool is_ferry = false);

        void clearGraph();

        std::vector<int64_t> dijkstra(int64_t current, int64_t target);
        RouteResult a_star(int64_t start, int64_t target);
        RouteResult bidirectional_a_star(int64_t start, int64_t target);
        crow::json::wvalue buildRouteJSON(const RouteResult &result);
        crow::json::wvalue getPathAsJSON(int64_t current, int64_t target);

        // Binary graph cache
        bool saveGraphCache(const std::string &cache_path);
        bool loadGraphCache(const std::string &cache_path, float min_lat, float max_lat, float min_lon, float max_lon);

        ~Graph()
        {
            app.stop();
        }

        void startServer();
        int64_t findNearestNode(float lat, float lon);
        int64_t findNearestReachableNode(float lat, float lon, const std::unordered_set<int64_t> &reachable);
    };
} // namespace graph
#endif // GRAPH_HPP_
