
#include "graph.hpp"
#include "utils.hpp"

#include <queue>
#include <unordered_set>
#include <fstream>
#include <cstring>
#include <sstream>
#include <nlohmann/json.hpp>

namespace graph
{
    static uint64_t readCurrentRssBytes()
    {
        std::ifstream status("/proc/self/status");
        if (!status)
            return 0;

        std::string line;
        while (std::getline(status, line))
        {
            if (line.rfind("VmRSS:", 0) == 0)
            {
                std::istringstream iss(line);
                std::string key;
                uint64_t kb = 0;
                std::string unit;
                iss >> key >> kb >> unit;
                return kb * 1024ULL;
            }
        }
        return 0;
    }

    void Graph::addNode(int64_t id, float lat, float lon)
    {
        // Only add the node if it doesn't already exist
        nodes.try_emplace(id, Node{lat, lon});
    }
    void Graph::addEdge(int64_t u, int64_t v, float speed_kmh, bool oneway, bool is_ferry)
    {
        auto it_u = nodes.find(u);
        auto it_v = nodes.find(v);
        if (it_u != nodes.end() && it_v != nodes.end())
        {
            float distance_m = utils::haversine(it_u->second.lat, it_u->second.lon, it_v->second.lat, it_v->second.lon);
            float time_s;
            if (is_ferry)
            {
                // Ferry: no traffic factor, but add 30 min waiting penalty per ferry way
                // The penalty is spread across segments proportionally, but we add
                // a large fixed cost on the first segment to discourage multiple ferries
                constexpr float ferry_boarding_penalty_s = 10.0f * 60.0f; // 10 minutes
                time_s = (distance_m / (speed_kmh / 3.6f)) + ferry_boarding_penalty_s;
            }
            else
            {
                constexpr float traffic_factor = 1.25f;
                time_s = (distance_m / (speed_kmh / 3.6f)) * traffic_factor;
            }
            adjList[u].push_back({v, time_s});
            reverseAdj[v].push_back({u, time_s});
            incoming_count[v]++;
            if (!oneway)
            {
                adjList[v].push_back({u, time_s});
                reverseAdj[u].push_back({v, time_s});
                incoming_count[u]++;
            }
        }
    }

    // clear the graph if necessary
    void Graph::clearGraph()
    {
        nodes.clear();
        adjList.clear();
        reverseAdj.clear();
        incoming_count.clear();
        spatial_grid.clear();
        well_connected_nodes.clear();
        uf_parent.clear();
        uf_rank.clear();
        largest_component_root = -1;
        loaded_area = LoadedArea{};
    }

    void Graph::ensureAreaLoaded(float start_lat, float start_lon, float end_lat, float end_lon, float margin_scale, bool force)
    {
        if (startup_preloaded && !force)
        {
            std::cout << "Startup-preloaded graph active, skipping dynamic reload." << std::endl;
            return;
        }

        // Calculate what the needed area would be
        float lat_min = std::min(start_lat, end_lat);
        float lat_max = std::max(start_lat, end_lat);
        float lon_min = std::min(start_lon, end_lon);
        float lon_max = std::max(start_lon, end_lon);

        float lat_span = lat_max - lat_min;
        float lon_span = lon_max - lon_min;
        float lat_margin = std::max(lat_span * 0.75f, 1.0f) * margin_scale;
        float lon_margin = std::max(lon_span * 0.75f, 2.0f) * margin_scale;

        float min_lat = std::max(lat_min - lat_margin, -90.0f);
        float max_lat = std::min(lat_max + lat_margin, 90.0f);
        float min_lon = std::max(lon_min - lon_margin, -180.0f);
        float max_lon = std::min(lon_max + lon_margin, 180.0f);

        float needed_area = (max_lat - min_lat) * (max_lon - min_lon);

        // Check if both points are within the already loaded area
        if (!force && loaded_area.contains(start_lat, start_lon) && loaded_area.contains(end_lat, end_lon))
        {
            // Only reload if loaded area is extremely disproportionate (>10x)
            // This avoids wasteful reloads when preload just loaded for a similar route
            if (loaded_area.area() > needed_area * 10.0f)
            {
                std::cout << "Loaded area (" << loaded_area.area() << " sq.deg) is much larger than needed ("
                          << needed_area << " sq.deg) — reloading optimized area." << std::endl;
            }
            else
            {
                std::cout << "Area already loaded, skipping reload." << std::endl;
                return;
            }
        }

        std::cout << "Loading new area for route (margin_scale=" << margin_scale << ")..." << std::endl;
        std::cout << "Loading area: lat [" << min_lat << ", " << max_lat
                  << "] lon [" << min_lon << ", " << max_lon << "]" << std::endl;

        // Clear existing graph data and reload
        nodes.clear();
        adjList.clear();
        reverseAdj.clear();
        incoming_count.clear();
        spatial_grid.clear();
        well_connected_nodes.clear();
        std::unordered_map<int64_t, Node>().swap(nodes);
        std::unordered_map<int64_t, std::vector<Edge>>().swap(adjList);
        std::unordered_map<int64_t, std::vector<Edge>>().swap(reverseAdj);
        std::unordered_map<int64_t, uint16_t>().swap(incoming_count);

        // Try loading from binary cache first
        std::string cache_path = osm_file_path + ".navgraph.cache";
        bool loaded_from_cache = loadGraphCache(cache_path, min_lat, max_lat, min_lon, max_lon);

        if (!loaded_from_cache)
        {
            utils::loadTile(*this, min_lon, min_lat, max_lon, max_lat, osm_file_path);
            loaded_area = {min_lat, max_lat, min_lon, max_lon, true};
            std::cout << "Area loaded: " << nodes.size() << " nodes" << std::endl;

            // Save cache for next time
            saveGraphCache(cache_path);
        }

        // Build spatial grid for fast nearest-node lookups
        buildSpatialGrid();

        // Build connected components for instant reachability checks
        buildComponents();

        // Build well-connected node set (bidirectional reachability via BFS)
        buildWellConnectedSet();
    }

    void Graph::preloadDataForFastRouting(float target_memory_gb)
    {
        if (osm_file_path.empty())
        {
            throw std::runtime_error("OSM file path is not set");
        }

        std::cout << "Startup preload: loading graph for fast routing/search..." << std::endl;
        auto t0 = high_resolution_clock::now();

        // Load the full drivable graph once at startup.
        constexpr float full_min_lat = -90.0f;
        constexpr float full_max_lat = 90.0f;
        constexpr float full_min_lon = -180.0f;
        constexpr float full_max_lon = 180.0f;

        clearGraph();

        std::string cache_path = osm_file_path + ".navgraph.cache";
        bool loaded_from_cache = loadGraphCache(
            cache_path,
            full_min_lat,
            full_max_lat,
            full_min_lon,
            full_max_lon);

        if (!loaded_from_cache)
        {
            utils::loadTile(*this, full_min_lon, full_min_lat, full_max_lon, full_max_lat, osm_file_path);
            loaded_area = {full_min_lat, full_max_lat, full_min_lon, full_max_lon, true};
            saveGraphCache(cache_path);
        }

        // Ensure all fast-query structures are present regardless of cache path.
        buildSpatialGrid();
        buildComponents();
        buildWellConnectedSet();

        startup_preloaded = true;

        auto t1 = high_resolution_clock::now();
        uint64_t rss_bytes = readCurrentRssBytes();
        const double rss_gb = static_cast<double>(rss_bytes) / (1024.0 * 1024.0 * 1024.0);

        std::cout << "Startup preload complete in "
                  << duration_cast<milliseconds>(t1 - t0).count() << "ms" << std::endl;
        if (rss_bytes > 0)
        {
            std::cout << "Backend RSS after preload: " << rss_gb << " GB"
                      << " (target: " << target_memory_gb << " GB)" << std::endl;
            if (rss_gb > static_cast<double>(target_memory_gb))
            {
                std::cout << "[WARN] RSS is above target. Consider reducing retained data/index detail."
                          << std::endl;
            }
        }
    }

    void Graph::buildSpatialGrid()
    {
        auto t0 = high_resolution_clock::now();
        spatial_grid.clear();
        for (const auto &[id, node] : nodes)
        {
            // Only index nodes that have edges (are part of the road network)
            if (adjList.count(id))
            {
                auto key = toGridKey(node.lat, node.lon);
                spatial_grid[key].push_back(id);
            }
        }
        auto t1 = high_resolution_clock::now();
        std::cout << "Spatial grid built: " << spatial_grid.size() << " cells in "
                  << duration_cast<milliseconds>(t1 - t0).count() << "ms" << std::endl;
    }

    size_t Graph::reverseReachCount(int64_t node, size_t max_depth, size_t max_nodes)
    {
        // BFS backwards from node using reverseAdj — how many nodes can reach this node?
        std::unordered_set<int64_t> visited;
        std::queue<std::pair<int64_t, size_t>> q; // (node, depth)
        q.push({node, 0});
        visited.insert(node);

        while (!q.empty() && visited.size() < max_nodes)
        {
            auto [cur, depth] = q.front();
            q.pop();
            if (depth >= max_depth)
                continue;

            auto rit = reverseAdj.find(cur);
            if (rit == reverseAdj.end())
                continue;
            for (const auto &e : rit->second)
            {
                if (visited.insert(e.to).second)
                {
                    q.push({e.to, depth + 1});
                }
            }
        }
        return visited.size();
    }

    void Graph::buildWellConnectedSet()
    {
        auto t0 = high_resolution_clock::now();
        well_connected_nodes.clear();

        // Strategy: BFS forward from a central hub node in the largest component,
        // then BFS backward (reverse) from the same hub.
        // Intersection = strongly connected component (nodes mutually reachable).

        // Pick a hub: find a node near the geographic center of the loaded area that's in the largest component
        if (largest_component_root == -1)
            return;

        float center_lat = (loaded_area.min_lat + loaded_area.max_lat) / 2.0f;
        float center_lon = (loaded_area.min_lon + loaded_area.max_lon) / 2.0f;

        // Find nearest node in largest component to center
        int64_t hub = -1;
        float hub_dist = std::numeric_limits<float>::max();
        auto center_key = toGridKey(center_lat, center_lon);
        for (int ring = 0; ring <= 50; ring++)
        {
            for (int dlat = -ring; dlat <= ring; dlat++)
            {
                for (int dlon = -ring; dlon <= ring; dlon++)
                {
                    if (ring > 0 && std::abs(dlat) < ring && std::abs(dlon) < ring)
                        continue;
                    GridKey key = {center_key.lat_idx + dlat, center_key.lon_idx + dlon};
                    auto git = spatial_grid.find(key);
                    if (git == spatial_grid.end())
                        continue;
                    for (int64_t id : git->second)
                    {
                        if (uf_parent.count(id) && ufFind(id) == largest_component_root)
                        {
                            float d = utils::haversine(center_lat, center_lon, nodes[id].lat, nodes[id].lon);
                            if (d < hub_dist)
                            {
                                hub_dist = d;
                                hub = id;
                            }
                        }
                    }
                }
            }
            if (hub != -1)
                break;
        }

        if (hub == -1)
        {
            std::cout << "Could not find hub node for well-connected set" << std::endl;
            return;
        }

        std::cout << "Building well-connected set from hub node " << hub << "..." << std::endl;

        // Forward BFS from hub
        std::unordered_set<int64_t> forward_set;
        {
            std::queue<int64_t> q;
            q.push(hub);
            forward_set.insert(hub);
            while (!q.empty())
            {
                int64_t cur = q.front();
                q.pop();
                auto ait = adjList.find(cur);
                if (ait == adjList.end())
                    continue;
                for (const auto &e : ait->second)
                {
                    if (forward_set.insert(e.to).second)
                        q.push(e.to);
                }
            }
        }
        std::cout << "Forward BFS: " << forward_set.size() << " nodes reachable from hub" << std::endl;

        // Backward BFS from hub (using reverse adjacency)
        std::unordered_set<int64_t> backward_set;
        {
            std::queue<int64_t> q;
            q.push(hub);
            backward_set.insert(hub);
            while (!q.empty())
            {
                int64_t cur = q.front();
                q.pop();
                auto rit = reverseAdj.find(cur);
                if (rit == reverseAdj.end())
                    continue;
                for (const auto &e : rit->second)
                {
                    if (backward_set.insert(e.to).second)
                        q.push(e.to);
                }
            }
        }
        std::cout << "Backward BFS: " << backward_set.size() << " nodes can reach hub" << std::endl;

        // Intersection = strongly connected component
        for (int64_t id : forward_set)
        {
            if (backward_set.count(id))
                well_connected_nodes.insert(id);
        }

        auto t1 = high_resolution_clock::now();
        std::cout << "Well-connected set built: " << well_connected_nodes.size() << " nodes (SCC) in "
                  << duration_cast<milliseconds>(t1 - t0).count() << "ms" << std::endl;
    }

    int64_t Graph::findNearestInGrid(float lat, float lon)
    {
        // Search expanding rings of grid cells
        auto center = toGridKey(lat, lon);
        constexpr int MAX_RING = 20; // Max search radius in cells (~20km)

        int64_t best_node = -1;
        float best_dist = std::numeric_limits<float>::max();
        int64_t best_node_scc = -1; // Best node in strongly connected component
        float best_dist_scc = std::numeric_limits<float>::max();

        for (int ring = 0; ring <= MAX_RING; ring++)
        {
            for (int dlat = -ring; dlat <= ring; dlat++)
            {
                for (int dlon = -ring; dlon <= ring; dlon++)
                {
                    if (ring > 0 && std::abs(dlat) < ring && std::abs(dlon) < ring)
                        continue;

                    GridKey key = {center.lat_idx + dlat, center.lon_idx + dlon};
                    auto git = spatial_grid.find(key);
                    if (git == spatial_grid.end())
                        continue;

                    for (int64_t id : git->second)
                    {
                        const auto &n = nodes[id];
                        float d = utils::haversine(lat, lon, n.lat, n.lon);

                        auto ait = adjList.find(id);
                        size_t out_edges = (ait != adjList.end()) ? ait->second.size() : 0;

                        // Track best overall
                        if (d < best_dist && (out_edges >= 1 || best_node == -1))
                        {
                            best_dist = d;
                            best_node = id;
                        }

                        // Track best node in the SCC (strongly connected — guaranteed mutually reachable)
                        if (!well_connected_nodes.empty() && well_connected_nodes.count(id) && d < best_dist_scc)
                        {
                            best_dist_scc = d;
                            best_node_scc = id;
                        }
                    }
                }
            }
            // Prefer SCC node
            if (best_node_scc != -1 && ring > 0)
                break;
            if (best_node != -1 && ring > 0 && best_node_scc == -1)
                break;
        }

        // Prefer SCC node (within 500m)
        if (best_node_scc != -1 && (best_dist_scc < 500.0f || best_node == -1))
            return best_node_scc;
        return best_node;
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
            if (nodes.find(at) == nodes.end())
            {
                std::cerr << "Error: Node " << at << " is not found" << std::endl;
                return {};
            }
            path.push_back(at);
        }

        path.push_back(current);
        reverse(path.begin(), path.end());
        return path;
    }

    // A* algorithm:
    RouteResult Graph::a_star(int64_t start, int64_t target)
    {
        std::cout << "Running A* algo" << std::endl;

        // Cache target node coordinates to avoid repeated map lookups
        const float target_lat = nodes[target].lat;
        const float target_lon = nodes[target].lon;

        auto heuristic = [&, target_lat, target_lon](int64_t from)
        {
            const auto &n = nodes[from];
            // Heuristic: travel time assuming max speed (110 km/h) — admissible lower bound
            float distance_m = utils::haversine(n.lat, n.lon, target_lat, target_lon);
            return distance_m / (110.0f / 3.6f); // seconds (no traffic factor — must be admissible)
        };

        using PQEntry = std::pair<float, int64_t>; // (f_score, node)
        std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> pq;

        std::unordered_map<int64_t, float> g_score;    // Distance from start (lazy: missing = INFINITY)
        std::unordered_map<int64_t, int64_t> previous; // Path reconstruction
        std::unordered_set<int64_t> visited;           // Skip already-settled nodes

        // Track the closest-to-target node (for fallback if target unreachable)
        int64_t closest_node = start;
        float closest_dist = utils::haversine(nodes[start].lat, nodes[start].lon, target_lat, target_lon);

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

            // Update closest-to-target tracker
            const auto &cn = nodes[current];
            float dist_to_target = utils::haversine(cn.lat, cn.lon, target_lat, target_lon);
            if (dist_to_target < closest_dist)
            {
                closest_dist = dist_to_target;
                closest_node = current;
            }

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
        {
            std::cout << "A* explored " << visited.size() << " nodes out of " << nodes.size() << " — target not reachable" << std::endl;

            // Auto-fallback: reconstruct path to the closest reachable node
            if (closest_node != start && g_score.count(closest_node))
            {
                const auto &cn = nodes[closest_node];
                std::cout << "[A* FALLBACK] Using closest reachable node " << closest_node
                          << " (" << closest_dist << "m from target, lat=" << cn.lat << " lon=" << cn.lon << ")" << std::endl;

                float fallback_time = g_score[closest_node];
                std::vector<int64_t> path;
                for (int64_t at = closest_node; at != start; at = previous[at])
                {
                    path.push_back(at);
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());

                std::cout << "Estimated travel time: " << fallback_time << " seconds" << std::endl;
                return {path, fallback_time};
            }

            return {{}, 0.0f, std::move(visited)}; // Return visited set for last-resort fallback
        }

        float total_time = it->second;

        // Reconstruct path:
        std::vector<int64_t> path;
        for (int64_t at = target; at != start; at = previous[at])
        {
            path.push_back(at);
        }

        path.push_back(start);
        std::reverse(path.begin(), path.end());
        return {path, total_time};
    }

    // Bidirectional A* — searches from both start and target, meets in the middle
    RouteResult Graph::bidirectional_a_star(int64_t start, int64_t target)
    {
        std::cout << "Running Bidirectional A*" << std::endl;

        if (start == target)
            return {{start}, 0.0f};

        const float start_lat = nodes[start].lat, start_lon = nodes[start].lon;
        const float target_lat = nodes[target].lat, target_lon = nodes[target].lon;
        constexpr float max_speed_ms = 110.0f / 3.6f;

        // Forward heuristic: estimate to target
        auto h_forward = [&](int64_t n) -> float
        {
            const auto &nd = nodes[n];
            return utils::haversine(nd.lat, nd.lon, target_lat, target_lon) / max_speed_ms;
        };
        // Backward heuristic: estimate to start
        auto h_backward = [&](int64_t n) -> float
        {
            const auto &nd = nodes[n];
            return utils::haversine(nd.lat, nd.lon, start_lat, start_lon) / max_speed_ms;
        };

        using PQEntry = std::pair<float, int64_t>;
        std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<>> pq_f, pq_b;

        const size_t expected = nodes.size() / 4;
        std::unordered_map<int64_t, float> g_f, g_b;
        std::unordered_map<int64_t, int64_t> prev_f, prev_b;
        g_f.reserve(expected);
        g_b.reserve(expected);
        prev_f.reserve(expected);
        prev_b.reserve(expected);

        // Use g_score sentinel: if node not in g_f/g_b, it hasn't been reached
        // Use closed flag via separate value: INFINITY g_score = not reached

        g_f[start] = 0;
        g_b[target] = 0;
        pq_f.push({h_forward(start), start});
        pq_b.push({h_backward(target), target});

        float mu = std::numeric_limits<float>::max(); // Best known path cost
        int64_t meeting_node = -1;

        // Track closest-to-target for fallback (use cached haversine distance)
        int64_t closest_node = start;
        float closest_dist = utils::haversine(start_lat, start_lon, target_lat, target_lon);

        size_t total_explored = 0;
        bool forward_turn = true;

        while (!pq_f.empty() || !pq_b.empty())
        {
            // Termination check
            float min_f = pq_f.empty() ? std::numeric_limits<float>::max() : pq_f.top().first;
            float min_b = pq_b.empty() ? std::numeric_limits<float>::max() : pq_b.top().first;
            if (min_f >= mu && min_b >= mu)
                break;

            // Alternate between forward and backward, or pick whichever is available
            bool do_forward;
            if (pq_f.empty())
                do_forward = false;
            else if (pq_b.empty())
                do_forward = true;
            else
                do_forward = (min_f <= min_b); // Expand the cheaper side

            if (do_forward && min_f < mu)
            {
                auto [f, cur] = pq_f.top();
                pq_f.pop();

                // Skip stale entries
                auto git_cur = g_f.find(cur);
                if (git_cur == g_f.end())
                    continue;
                float cur_g = git_cur->second;
                if (cur_g + h_forward(cur) > f + 0.001f)
                    continue; // stale

                total_explored++;

                // Check meeting when settling (not on every edge)
                auto bit = g_b.find(cur);
                if (bit != g_b.end())
                {
                    float total = cur_g + bit->second;
                    if (total < mu)
                    {
                        mu = total;
                        meeting_node = cur;
                    }
                }

                // Update closest tracker
                const auto &cn = nodes[cur];
                float dt = utils::haversine(cn.lat, cn.lon, target_lat, target_lon);
                if (dt < closest_dist)
                {
                    closest_dist = dt;
                    closest_node = cur;
                }

                for (const auto &[neighbor, weight] : adjList[cur])
                {
                    float tent_g = cur_g + weight;
                    auto nit = g_f.find(neighbor);
                    if (nit == g_f.end() || tent_g < nit->second)
                    {
                        g_f[neighbor] = tent_g;
                        prev_f[neighbor] = cur;
                        pq_f.push({tent_g + h_forward(neighbor), neighbor});
                    }
                }
            }
            else if (!do_forward && min_b < mu)
            {
                auto [f, cur] = pq_b.top();
                pq_b.pop();

                auto git_cur = g_b.find(cur);
                if (git_cur == g_b.end())
                    continue;
                float cur_g = git_cur->second;
                if (cur_g + h_backward(cur) > f + 0.001f)
                    continue; // stale

                total_explored++;

                // Check meeting when settling
                auto fit = g_f.find(cur);
                if (fit != g_f.end())
                {
                    float total = cur_g + fit->second;
                    if (total < mu)
                    {
                        mu = total;
                        meeting_node = cur;
                    }
                }

                auto rit = reverseAdj.find(cur);
                if (rit != reverseAdj.end())
                {
                    for (const auto &[neighbor, weight] : rit->second)
                    {
                        float tent_g = cur_g + weight;
                        auto nit = g_b.find(neighbor);
                        if (nit == g_b.end() || tent_g < nit->second)
                        {
                            g_b[neighbor] = tent_g;
                            prev_b[neighbor] = cur;
                            pq_b.push({tent_g + h_backward(neighbor), neighbor});
                        }
                    }
                }
            }
            else
            {
                // Both sides exhausted or above mu — pop whichever is available to drain
                if (!pq_f.empty())
                    pq_f.pop();
                else if (!pq_b.empty())
                    pq_b.pop();
                else
                    break;
            }
        }

        std::cout << "Bidirectional A* explored " << total_explored << " nodes" << std::endl;

        if (meeting_node == -1)
        {
            std::cout << "Target not reachable" << std::endl;

            // Fallback: path to closest reachable node
            if (closest_node != start && g_f.count(closest_node))
            {
                std::cout << "[BIDIR FALLBACK] Using closest node " << closest_node
                          << " (" << closest_dist << "m from target)" << std::endl;
                float fallback_time = g_f[closest_node];
                std::vector<int64_t> path;
                for (int64_t at = closest_node; at != start; at = prev_f[at])
                    path.push_back(at);
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                std::cout << "Estimated travel time: " << fallback_time << " seconds" << std::endl;
                return {path, fallback_time};
            }
            return {{}, 0.0f};
        }

        // Reconstruct path: start → meeting_node → target
        std::vector<int64_t> path;

        // Forward part: start → meeting
        {
            std::vector<int64_t> fwd;
            for (int64_t at = meeting_node; at != start; at = prev_f[at])
                fwd.push_back(at);
            fwd.push_back(start);
            std::reverse(fwd.begin(), fwd.end());
            path = std::move(fwd);
        }

        // Backward part: meeting → target (follow prev_b)
        {
            int64_t at = meeting_node;
            while (at != target)
            {
                at = prev_b[at];
                path.push_back(at);
            }
        }

        std::cout << "Estimated travel time: " << mu << " seconds" << std::endl;
        return {path, mu};
    }

    // Binary graph cache — save/load the parsed graph to skip PBF parsing
    bool Graph::saveGraphCache(const std::string &cache_path)
    {
        auto t0 = high_resolution_clock::now();
        std::ofstream out(cache_path, std::ios::binary);
        if (!out)
        {
            std::cerr << "Cannot open cache file for writing: " << cache_path << std::endl;
            return false;
        }

        // Header: magic + version + bounding box
        const uint32_t magic = 0x4E415647; // "NAVG"
        const uint32_t version = 1;
        out.write(reinterpret_cast<const char *>(&magic), sizeof(magic));
        out.write(reinterpret_cast<const char *>(&version), sizeof(version));
        out.write(reinterpret_cast<const char *>(&loaded_area.min_lat), sizeof(float));
        out.write(reinterpret_cast<const char *>(&loaded_area.max_lat), sizeof(float));
        out.write(reinterpret_cast<const char *>(&loaded_area.min_lon), sizeof(float));
        out.write(reinterpret_cast<const char *>(&loaded_area.max_lon), sizeof(float));

        // Nodes: count, then [id, lat, lon] for each
        uint64_t node_count = nodes.size();
        out.write(reinterpret_cast<const char *>(&node_count), sizeof(node_count));
        for (const auto &[id, node] : nodes)
        {
            out.write(reinterpret_cast<const char *>(&id), sizeof(id));
            out.write(reinterpret_cast<const char *>(&node.lat), sizeof(float));
            out.write(reinterpret_cast<const char *>(&node.lon), sizeof(float));
        }

        // Edges: count entries in adjList, then for each: [from_id, edge_count, [to, weight]...]
        uint64_t adj_count = adjList.size();
        out.write(reinterpret_cast<const char *>(&adj_count), sizeof(adj_count));
        for (const auto &[id, edges] : adjList)
        {
            out.write(reinterpret_cast<const char *>(&id), sizeof(id));
            uint32_t edge_count = edges.size();
            out.write(reinterpret_cast<const char *>(&edge_count), sizeof(edge_count));
            for (const auto &e : edges)
            {
                out.write(reinterpret_cast<const char *>(&e.to), sizeof(e.to));
                out.write(reinterpret_cast<const char *>(&e.weight), sizeof(e.weight));
            }
        }

        // Reverse edges
        uint64_t rev_count = reverseAdj.size();
        out.write(reinterpret_cast<const char *>(&rev_count), sizeof(rev_count));
        for (const auto &[id, edges] : reverseAdj)
        {
            out.write(reinterpret_cast<const char *>(&id), sizeof(id));
            uint32_t edge_count = edges.size();
            out.write(reinterpret_cast<const char *>(&edge_count), sizeof(edge_count));
            for (const auto &e : edges)
            {
                out.write(reinterpret_cast<const char *>(&e.to), sizeof(e.to));
                out.write(reinterpret_cast<const char *>(&e.weight), sizeof(e.weight));
            }
        }

        out.close();
        auto t1 = high_resolution_clock::now();
        std::cout << "Graph cache saved to " << cache_path << " in "
                  << duration_cast<milliseconds>(t1 - t0).count() << "ms" << std::endl;
        return true;
    }

    bool Graph::loadGraphCache(const std::string &cache_path, float min_lat, float max_lat, float min_lon, float max_lon)
    {
        auto t0 = high_resolution_clock::now();
        std::ifstream in(cache_path, std::ios::binary);
        if (!in)
            return false;

        // Verify header
        uint32_t magic, version;
        in.read(reinterpret_cast<char *>(&magic), sizeof(magic));
        in.read(reinterpret_cast<char *>(&version), sizeof(version));
        if (magic != 0x4E415647 || version != 1)
        {
            std::cout << "Cache file invalid" << std::endl;
            return false;
        }

        // Read bounding box and verify it covers what we need
        float cached_min_lat, cached_max_lat, cached_min_lon, cached_max_lon;
        in.read(reinterpret_cast<char *>(&cached_min_lat), sizeof(float));
        in.read(reinterpret_cast<char *>(&cached_max_lat), sizeof(float));
        in.read(reinterpret_cast<char *>(&cached_min_lon), sizeof(float));
        in.read(reinterpret_cast<char *>(&cached_max_lon), sizeof(float));

        // Check if cache covers the needed area
        if (cached_min_lat > min_lat || cached_max_lat < max_lat ||
            cached_min_lon > min_lon || cached_max_lon < max_lon)
        {
            std::cout << "Cache bounds don't cover needed area — reparsing" << std::endl;
            return false;
        }

        // --- Bulk-read nodes into contiguous buffer, then filter ---
        // Each node record: int64_t id (8) + float lat (4) + float lon (4) = 16 bytes
        uint64_t node_count;
        in.read(reinterpret_cast<char *>(&node_count), sizeof(node_count));

        constexpr size_t NODE_RECORD_SIZE = sizeof(int64_t) + 2 * sizeof(float); // 16 bytes
        std::vector<char> node_buf(node_count * NODE_RECORD_SIZE);
        in.read(node_buf.data(), node_buf.size());

        nodes.reserve(node_count);
        uint64_t nodes_skipped = 0;
        const char *ptr = node_buf.data();
        for (uint64_t i = 0; i < node_count; i++, ptr += NODE_RECORD_SIZE)
        {
            int64_t id;
            float lat, lon;
            std::memcpy(&id, ptr, sizeof(int64_t));
            std::memcpy(&lat, ptr + sizeof(int64_t), sizeof(float));
            std::memcpy(&lon, ptr + sizeof(int64_t) + sizeof(float), sizeof(float));
            if (lat >= min_lat && lat <= max_lat && lon >= min_lon && lon <= max_lon)
                nodes[id] = {lat, lon};
            else
                nodes_skipped++;
        }
        node_buf.clear();
        node_buf.shrink_to_fit(); // Free buffer memory

        if (nodes_skipped > 0)
            std::cout << "Cache bbox filter: kept " << nodes.size() << " nodes, skipped " << nodes_skipped << std::endl;

        // --- Bulk-read entire edge section into memory, then parse ---
        auto edge_start_pos = in.tellg();
        in.seekg(0, std::ios::end);
        auto file_end = in.tellg();
        size_t remaining = file_end - edge_start_pos;
        in.seekg(edge_start_pos);

        std::vector<char> edge_buf(remaining);
        in.read(edge_buf.data(), remaining);
        in.close();

        const char *ep = edge_buf.data();

        // Helper lambdas to read from buffer
        auto read_u64 = [&]() -> uint64_t
        { uint64_t v; std::memcpy(&v, ep, 8); ep += 8; return v; };
        auto read_i64 = [&]() -> int64_t
        { int64_t v;  std::memcpy(&v, ep, 8); ep += 8; return v; };
        auto read_u32 = [&]() -> uint32_t
        { uint32_t v; std::memcpy(&v, ep, 4); ep += 4; return v; };
        auto read_f32 = [&]() -> float
        { float v;    std::memcpy(&v, ep, 4); ep += 4; return v; };

        // Read forward edges
        uint64_t adj_count = read_u64();
        adjList.reserve(nodes.size());
        for (uint64_t i = 0; i < adj_count; i++)
        {
            int64_t id = read_i64();
            uint32_t edge_count = read_u32();
            bool from_in_bbox = nodes.count(id);
            for (uint32_t j = 0; j < edge_count; j++)
            {
                int64_t to = read_i64();
                float weight = read_f32();
                if (from_in_bbox && nodes.count(to))
                    adjList[id].push_back({to, weight});
            }
        }

        // Read reverse edges
        uint64_t rev_count = read_u64();
        reverseAdj.reserve(nodes.size());
        for (uint64_t i = 0; i < rev_count; i++)
        {
            int64_t id = read_i64();
            uint32_t edge_count = read_u32();
            bool from_in_bbox = nodes.count(id);
            for (uint32_t j = 0; j < edge_count; j++)
            {
                int64_t to = read_i64();
                float weight = read_f32();
                if (from_in_bbox && nodes.count(to))
                    reverseAdj[id].push_back({to, weight});
            }
        }

        edge_buf.clear();
        edge_buf.shrink_to_fit(); // Free buffer memory

        // Rebuild incoming_count from reverseAdj
        for (const auto &[id, edges] : reverseAdj)
        {
            incoming_count[id] = edges.size();
        }

        // Set loaded_area to the REQUESTED bbox, not the cache bbox
        loaded_area = {min_lat, max_lat, min_lon, max_lon, true};

        auto t1 = high_resolution_clock::now();
        std::cout << "Graph cache loaded: " << nodes.size() << " nodes in "
                  << duration_cast<milliseconds>(t1 - t0).count() << "ms" << std::endl;
        return true;
    }

    crow::json::wvalue Graph::buildRouteJSON(const RouteResult &result)
    {
        crow::json::wvalue response;

        if (result.path.empty())
        {
            response["error"] = "No route found";
            return response;
        }

        std::cout << "Estimated travel time: " << result.travel_time_seconds << " seconds" << std::endl;

        // Add travel time info
        int total_seconds = static_cast<int>(result.travel_time_seconds);
        int hours = total_seconds / 3600;
        int minutes = (total_seconds % 3600) / 60;
        response["travel_time_seconds"] = result.travel_time_seconds;
        response["travel_time_text"] = (hours > 0 ? std::to_string(hours) + " h " : "") + std::to_string(minutes) + " min";

        for (size_t i = 0; i < result.path.size(); i++)
        {
            auto it = nodes.find(result.path[i]);
            if (it != nodes.end())
            {
                response["path"][i]["id"] = result.path[i];
                response["path"][i]["lat"] = it->second.lat;
                response["path"][i]["lon"] = it->second.lon;
            }
        }
        return response;
    }

    crow::json::wvalue Graph::getPathAsJSON(int64_t current, int64_t target)
    {
        auto result = a_star(current, target);
        std::cout << "A* completed, path has " << result.path.size() << " nodes" << std::endl;
        return buildRouteJSON(result);
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

                            std::string msg_type = json_msg.value("type", "route_request");

                            if (msg_type == "preload")
                            {
                                // Preload: just load the area, don't calculate a route
                                float start_lat = json_msg["start"]["lat"];
                                float start_lon = json_msg["start"]["lon"];
                                float end_lat = json_msg["end"]["lat"];
                                float end_lon = json_msg["end"]["lon"];
                                std::cout << "Preloading area..." << std::endl;
                                {
                                    std::lock_guard<std::mutex> lock(graph_mutex);
                                    auto t_load_start = high_resolution_clock::now();
                                    ensureAreaLoaded(start_lat, start_lon, end_lat, end_lon);
                                    auto t_load_end = high_resolution_clock::now();
                                    std::cout << "Preload time: " << duration_cast<milliseconds>(t_load_end - t_load_start).count() << " ms" << std::endl;
                                }
                                std::cout << "Preload complete." << std::endl;
                                // Notify frontend that preload is done
                                conn.send_text("{\"preload_done\": true}");
                                return;
                            }

                            // Extract the start and end coordinates
                            float start_lat = json_msg["start"]["lat"];
                            float start_lon = json_msg["start"]["lon"];
                            float end_lat = json_msg["end"]["lat"];
                            float end_lon = json_msg["end"]["lon"];
                            std::cout << "Extracted coordinates successfully " << start_lat << " " << start_lon << " " << end_lat << " " << end_lon << std::endl;

                            // Lock the graph for the entire route calculation
                            std::lock_guard<std::mutex> lock(graph_mutex);

                            auto t_total_start = high_resolution_clock::now();

                            // Ensure the needed area is loaded
                            auto t_load_start = high_resolution_clock::now();
                            ensureAreaLoaded(start_lat, start_lon, end_lat, end_lon);
                            auto t_load_end = high_resolution_clock::now();
                            std::cout << "[TIMING] Area loading: " << duration_cast<milliseconds>(t_load_end - t_load_start).count() << " ms" << std::endl;

                            // Find the nearest nodes to the start and end coordinates
                            auto t_find_start = high_resolution_clock::now();
                            int64_t start_node = findNearestNode(start_lat, start_lon);
                            int64_t end_node = findNearestNode(end_lat, end_lon);
                            auto t_find_end = high_resolution_clock::now();
                            std::cout << "[TIMING] Find nearest nodes: " << duration_cast<milliseconds>(t_find_end - t_find_start).count() << " ms" << std::endl;
                            std::cout << "Found nearest nodes successfully" << std::endl;

                            // Log connectivity info
                            auto start_edges = adjList.find(start_node);
                            auto end_edges = adjList.find(end_node);
                            std::cout << "Start node " << start_node << " has " 
                                      << (start_edges != adjList.end() ? start_edges->second.size() : 0) 
                                      << " outgoing edges" << std::endl;
                            std::cout << "End node " << end_node << " has " 
                                      << (end_edges != adjList.end() ? end_edges->second.size() : 0) 
                                      << " outgoing edges" << std::endl;

                            // Pre-check connectivity using Union-Find
                            auto t_astar_start = high_resolution_clock::now();

                            if (!uf_parent.empty() && uf_parent.count(start_node) && uf_parent.count(end_node))
                            {
                                int64_t start_comp = ufFind(start_node);
                                int64_t end_comp = ufFind(end_node);
                                if (start_comp != end_comp)
                                {
                                    std::cout << "[COMPONENT] Start and end in different components — finding nearest same-component node" << std::endl;
                                    int64_t alt_end = findNearestInComponent(end_lat, end_lon, start_comp);
                                    if (alt_end != -1)
                                    {
                                        const auto &n = nodes[alt_end];
                                        float dist = utils::haversine(end_lat, end_lon, n.lat, n.lon);
                                        std::cout << "[COMPONENT] Replaced end node " << end_node << " with " << alt_end
                                                  << " (" << dist << "m away, same component)" << std::endl;
                                        end_node = alt_end;
                                    }
                                }
                            }

                            // Try to find a route (A* with haversine heuristic)
                            auto route_result = a_star(start_node, end_node);

                            // Fallback: if still no route (completely disconnected), try expanded area
                            if (route_result.path.empty())
                            {
                                std::cout << "[FALLBACK] No route found — retrying with expanded area (2x margin)..." << std::endl;
                                ensureAreaLoaded(start_lat, start_lon, end_lat, end_lon, 2.0f, true);
                                start_node = findNearestNode(start_lat, start_lon);
                                end_node = findNearestNode(end_lat, end_lon);
                                route_result = a_star(start_node, end_node);
                            }

                            // Build JSON from the already-computed route (no redundant A*)
                            crow::json::wvalue response = buildRouteJSON(route_result);

                            auto t_astar_end = high_resolution_clock::now();
                            std::cout << "[TIMING] A* route calculation: " << duration_cast<milliseconds>(t_astar_end - t_astar_start).count() << " ms" << std::endl;
                            std::cout << "Route completed, path has " << route_result.path.size() << " nodes" << std::endl;
                            auto t_total_end = high_resolution_clock::now();
                            std::cout << "[TIMING] Total route request: " << duration_cast<milliseconds>(t_total_end - t_total_start).count() << " ms" << std::endl;

                            conn.send_text(response.dump());
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "Error parsing JSON: " << e.what() << std::endl;
                        } })
            .onclose([](crow::websocket::connection &conn, const std::string &reason, uint16_t code)
                     { std::cout << "WebSocket closed. Reason: " << reason << ", Code: " << code << std::endl; });

        // Serve index.html at the root URL
        CROW_ROUTE(app, "/")
        ([]()
         {
            // Try multiple paths to find index.html regardless of working directory
            for (const auto &path : {"../index.html", "index.html", "../../index.html"})
            {
                std::ifstream f(path);
                if (f.good())
                {
                    std::string content((std::istreambuf_iterator<char>(f)),
                                         std::istreambuf_iterator<char>());
                    crow::response res(200);
                    res.set_header("Content-Type", "text/html; charset=utf-8");
                    res.body = content;
                    return res;
                }
            }
            return crow::response(404, "index.html not found"); });

        // Serve test-dynamic-route.html
        CROW_ROUTE(app, "/test-dynamic-route.html")
        ([]()
         {
            // Try multiple paths to find test-dynamic-route.html
            for (const auto &path : {"../test-dynamic-route.html", "test-dynamic-route.html", "../../test-dynamic-route.html"})
            {
                std::ifstream f(path);
                if (f.good())
                {
                    std::string content((std::istreambuf_iterator<char>(f)),
                                         std::istreambuf_iterator<char>());
                    crow::response res(200);
                    res.set_header("Content-Type", "text/html; charset=utf-8");
                    res.body = content;
                    return res;
                }
            }
            return crow::response(404, "test-dynamic-route.html not found"); });

        std::cout << "Server running on http://localhost:18080" << std::endl;
        app.port(18080).multithreaded().run();
    }

    int64_t Graph::ufFind(int64_t x)
    {
        while (uf_parent[x] != x)
        {
            uf_parent[x] = uf_parent[uf_parent[x]]; // path compression (halving)
            x = uf_parent[x];
        }
        return x;
    }

    void Graph::ufUnion(int64_t a, int64_t b)
    {
        a = ufFind(a);
        b = ufFind(b);
        if (a == b)
            return;
        // Union by rank
        if (uf_rank[a] < uf_rank[b])
            std::swap(a, b);
        uf_parent[b] = a;
        if (uf_rank[a] == uf_rank[b])
            uf_rank[a]++;
    }

    void Graph::buildComponents()
    {
        auto t0 = high_resolution_clock::now();
        uf_parent.clear();
        uf_rank.clear();

        // Initialize: every node with edges is its own parent
        for (const auto &[id, edges] : adjList)
        {
            uf_parent[id] = id;
            uf_rank[id] = 0;
        }

        // Union all connected edges
        for (const auto &[id, edges] : adjList)
        {
            for (const auto &e : edges)
            {
                if (uf_parent.count(e.to))
                    ufUnion(id, e.to);
            }
        }

        // Count component sizes
        std::unordered_map<int64_t, size_t> comp_sizes;
        for (const auto &[id, _] : uf_parent)
        {
            comp_sizes[ufFind(id)]++;
        }

        // Find largest component
        int64_t largest_root = -1;
        size_t largest_size = 0;
        for (const auto &[root, sz] : comp_sizes)
        {
            if (sz > largest_size)
            {
                largest_size = sz;
                largest_root = root;
            }
        }
        largest_component_root = largest_root;

        auto t1 = high_resolution_clock::now();
        std::cout << "Connected components built: " << comp_sizes.size() << " components, "
                  << "largest has " << largest_size << " nodes, in "
                  << duration_cast<milliseconds>(t1 - t0).count() << "ms" << std::endl;
    }

    int64_t Graph::findNearestInComponent(float lat, float lon, int64_t component_root)
    {
        // Use spatial grid but only return nodes in the same component
        auto center = toGridKey(lat, lon);
        constexpr int MAX_RING = 30;

        int64_t best_node = -1;
        float best_dist = std::numeric_limits<float>::max();

        for (int ring = 0; ring <= MAX_RING; ring++)
        {
            for (int dlat = -ring; dlat <= ring; dlat++)
            {
                for (int dlon = -ring; dlon <= ring; dlon++)
                {
                    if (ring > 0 && std::abs(dlat) < ring && std::abs(dlon) < ring)
                        continue;

                    GridKey key = {center.lat_idx + dlat, center.lon_idx + dlon};
                    auto git = spatial_grid.find(key);
                    if (git == spatial_grid.end())
                        continue;

                    for (int64_t id : git->second)
                    {
                        if (ufFind(id) != component_root)
                            continue;
                        const auto &n = nodes[id];
                        float d = utils::haversine(lat, lon, n.lat, n.lon);
                        if (d < best_dist)
                        {
                            auto ait = adjList.find(id);
                            size_t edges = (ait != adjList.end()) ? ait->second.size() : 0;
                            if (edges >= 2 || best_node == -1)
                            {
                                best_dist = d;
                                best_node = id;
                            }
                        }
                    }
                }
            }
            if (best_node != -1 && ring > 0)
                break;
        }
        return best_node;
    }

    int64_t Graph::findNearestNode(float lat, float lon)
    {
        // Use spatial grid if available (O(1) lookup instead of O(n))
        if (!spatial_grid.empty())
        {
            int64_t result = findNearestInGrid(lat, lon);
            if (result != -1)
            {
                const auto &n = nodes[result];
                auto adj_it = adjList.find(result);
                std::cout << "Nearest node (grid): " << result
                          << " with distance " << utils::haversine(lat, lon, n.lat, n.lon) << "m"
                          << " (" << (adj_it != adjList.end() ? adj_it->second.size() : 0)
                          << " edges)" << std::endl;
                return result;
            }
            std::cout << "Grid lookup failed, falling back to linear scan." << std::endl;
        }

        // Fallback: linear scan (only used if grid not built yet)
        constexpr size_t K = 10;
        constexpr size_t MIN_EDGES = 2;

        struct Candidate
        {
            int64_t id;
            float distance;
            size_t edge_count;
        };

        std::vector<Candidate> candidates;
        candidates.reserve(K + 1);

        for (const auto &[id, node] : nodes)
        {
            float distance = utils::haversine(lat, lon, node.lat, node.lon);

            if (candidates.size() < K || distance < candidates.back().distance)
            {
                auto adj_it = adjList.find(id);
                size_t edges = (adj_it != adjList.end()) ? adj_it->second.size() : 0;
                if (edges == 0)
                    continue;

                candidates.push_back({id, distance, edges});

                std::sort(candidates.begin(), candidates.end(),
                          [](const Candidate &a, const Candidate &b)
                          { return a.distance < b.distance; });
                if (candidates.size() > K)
                    candidates.resize(K);
            }
        }

        if (candidates.empty())
        {
            std::cerr << "Error: No reachable node found near (" << lat << ", " << lon << ")" << std::endl;
            return -1;
        }

        // Prefer the nearest well-connected node (>= MIN_EDGES)
        int64_t nearest_node = candidates[0].id;
        for (const auto &c : candidates)
        {
            if (c.edge_count >= MIN_EDGES)
            {
                nearest_node = c.id;
                break;
            }
        }

        const auto &n = nodes[nearest_node];
        auto adj_it = adjList.find(nearest_node);
        std::cout << "Nearest node: " << nearest_node
                  << " with distance " << utils::haversine(lat, lon, n.lat, n.lon)
                  << " (" << (adj_it != adjList.end() ? adj_it->second.size() : 0)
                  << " edges)" << std::endl;

        return nearest_node;
    }

    int64_t Graph::findNearestReachableNode(float lat, float lon, const std::unordered_set<int64_t> &reachable)
    {
        int64_t nearest_node = -1;
        float min_distance = std::numeric_limits<float>::max();

        for (const auto &node_id : reachable)
        {
            auto it = nodes.find(node_id);
            if (it == nodes.end())
                continue;
            float distance = utils::haversine(lat, lon, it->second.lat, it->second.lon);
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_node = node_id;
            }
        }

        if (nearest_node != -1)
        {
            const auto &n = nodes[nearest_node];
            std::cout << "Nearest reachable node: " << nearest_node
                      << " with distance " << min_distance << "m" << std::endl;
            std::cout << "Nearest reachable node coordinates: lat = "
                      << std::fixed << std::setprecision(8) << n.lat
                      << ", lon = " << n.lon << std::endl;
        }
        else
        {
            std::cerr << "No reachable node found near (" << lat << ", " << lon << ")" << std::endl;
        }

        return nearest_node;
    }
} // namespace graph
