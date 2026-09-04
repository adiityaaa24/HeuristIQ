#ifndef GRAPH_ASTAR_H
#define GRAPH_ASTAR_H

// A generic, header-only A* implementation for any graph represented as
// an adjacency list: NodeId -> vector<(neighbourId, edgeWeight)>.
//
// This is separate from AStarGrid (which is grid-specialised) to show
// A* applied to the general graph-search problem - e.g. road networks,
// state-space search, puzzle solvers - not just 2D grids.
//
// Usage:
//   GraphAStar<std::string> astar;
//   astar.addEdge("A", "B", 4.0);
//   astar.addEdge("B", "C", 2.0);
//   auto heuristic = [](const std::string& node) { return 0.0; }; // e.g. Dijkstra
//   auto result = astar.solve("A", "C", heuristic);

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <algorithm>
#include <functional>
#include <limits>

template <typename NodeId>
struct GraphAStarResult {
    bool found = false;
    std::vector<NodeId> path;
    double totalCost = 0.0;
    long long nodesExpanded = 0;
};

template <typename NodeId, typename Hash = std::hash<NodeId>>
class GraphAStar {
public:
    // Adds a directed edge. Call twice (both directions) for undirected graphs.
    void addEdge(const NodeId& from, const NodeId& to, double weight) {
        adjacency_[from].push_back({to, weight});
        adjacency_[to]; // ensure 'to' exists as a node even with no out-edges
    }

    // heuristic(node) should estimate remaining cost to the goal and must
    // be admissible (never overestimate) for A* to guarantee optimality.
    GraphAStarResult<NodeId> solve(
        const NodeId& start, const NodeId& goal,
        const std::function<double(const NodeId&)>& heuristic) const {

        GraphAStarResult<NodeId> result;

        using QueueItem = std::pair<double, NodeId>;
        auto cmp = [](const QueueItem& a, const QueueItem& b) {
            return a.first > b.first;
        };
        std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(cmp)> openSet(cmp);

        std::unordered_map<NodeId, double, Hash> gScore;
        std::unordered_map<NodeId, NodeId, Hash> cameFrom;
        std::unordered_set<NodeId, Hash> closed;

        gScore[start] = 0.0;
        openSet.push({heuristic(start), start});

        while (!openSet.empty()) {
            NodeId current = openSet.top().second;
            openSet.pop();

            if (closed.count(current)) continue;
            closed.insert(current);
            result.nodesExpanded++;

            if (current == goal) {
                result.found = true;
                result.totalCost = gScore[current];
                result.path = reconstruct(cameFrom, start, current);
                return result;
            }

            auto it = adjacency_.find(current);
            if (it == adjacency_.end()) continue;

            for (const auto& edge : it->second) {
                const NodeId& next = edge.first;
                double weight = edge.second;
                if (closed.count(next)) continue;

                double tentativeG = gScore[current] + weight;
                auto gIt = gScore.find(next);
                if (gIt == gScore.end() || tentativeG < gIt->second) {
                    gScore[next] = tentativeG;
                    cameFrom[next] = current;
                    openSet.push({tentativeG + heuristic(next), next});
                }
            }
        }
        return result; // found = false, no path exists
    }

private:
    std::unordered_map<NodeId, std::vector<std::pair<NodeId, double>>, Hash> adjacency_;

    static std::vector<NodeId> reconstruct(
        const std::unordered_map<NodeId, NodeId, Hash>& cameFrom,
        const NodeId& start, const NodeId& current) {
        std::vector<NodeId> path;
        NodeId node = current;
        path.push_back(node);
        while (!(node == start)) {
            auto it = cameFrom.find(node);
            if (it == cameFrom.end()) break;
            node = it->second;
            path.push_back(node);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

#endif // GRAPH_ASTAR_H
