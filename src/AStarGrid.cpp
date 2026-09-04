#include "AStarGrid.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>

AStarGrid::AStarGrid(const Grid& grid, bool allowDiagonal, Heuristic heuristic)
    : grid_(grid), allowDiagonal_(allowDiagonal), heuristic_(heuristic) {}

double AStarGrid::heuristicCost(const Point& a, const Point& b) const {
    double dr = std::abs(a.row - b.row);
    double dc = std::abs(a.col - b.col);

    switch (heuristic_) {
        case Heuristic::Manhattan:
            return dr + dc;
        case Heuristic::Euclidean:
            return std::sqrt(dr * dr + dc * dc);
        case Heuristic::Diagonal: {
            // Octile distance: admissible & consistent for 8-directional grids.
            double D = 1.0, D2 = std::sqrt(2.0);
            return D * (dr + dc) + (D2 - 2 * D) * std::min(dr, dc);
        }
        case Heuristic::Zero:
        default:
            return 0.0;
    }
}

std::vector<Point> AStarGrid::reconstructPath(
    const std::unordered_map<Point, Point, PointHash>& cameFrom,
    const Point& start, const Point& current) {
    std::vector<Point> path;
    Point node = current;
    path.push_back(node);
    while (!(node == start)) {
        auto it = cameFrom.find(node);
        if (it == cameFrom.end()) break; // safety guard
        node = it->second;
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

AStarResult AStarGrid::solve(const Point& start, const Point& goal) const {
    AStarResult result;
    auto t0 = std::chrono::high_resolution_clock::now();

    if (!grid_.isWalkable(start) || !grid_.isWalkable(goal)) {
        return result; // found = false
    }

    // Min-heap of (fScore, Point), ordered by fScore ascending.
    using QueueItem = std::pair<double, Point>;
    auto cmp = [](const QueueItem& a, const QueueItem& b) {
        return a.first > b.first; // smallest fScore has highest priority
    };
    std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(cmp)> openSet(cmp);

    std::unordered_map<Point, double, PointHash> gScore;
    std::unordered_map<Point, Point, PointHash> cameFrom;
    std::unordered_set<Point, PointHash> closed;

    gScore[start] = 0.0;
    openSet.push({heuristicCost(start, goal), start});

    while (!openSet.empty()) {
        Point current = openSet.top().second;
        openSet.pop();

        if (closed.count(current)) continue; // stale entry, skip
        closed.insert(current);
        result.visitedOrder.push_back(current);
        result.nodesExpanded++;

        if (current == goal) {
            result.found = true;
            result.path = reconstructPath(cameFrom, start, current);
            result.totalCost = gScore[current];
            break;
        }

        for (const Point& next : grid_.neighbours(current, allowDiagonal_)) {
            if (closed.count(next)) continue;

            // Step cost: diagonal moves cost sqrt(2) * cell weight,
            // orthogonal moves cost 1 * cell weight.
            bool diagonalStep = (next.row != current.row) && (next.col != current.col);
            double stepCost = (diagonalStep ? std::sqrt(2.0) : 1.0) * grid_.cost(next);
            double tentativeG = gScore[current] + stepCost;

            auto it = gScore.find(next);
            if (it == gScore.end() || tentativeG < it->second) {
                gScore[next] = tentativeG;
                cameFrom[next] = current;
                double f = tentativeG + heuristicCost(next, goal);
                openSet.push({f, next});
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    result.timeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}
