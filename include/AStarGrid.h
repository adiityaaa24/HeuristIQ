#ifndef ASTAR_GRID_H
#define ASTAR_GRID_H

#include "Grid.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>

enum class Heuristic {
    Manhattan,   // best for 4-directional movement
    Euclidean,   // straight-line distance
    Diagonal,    // best for 8-directional (octile) movement
    Zero         // turns A* into plain Dijkstra
};

// Result bundle returned by the solver: the path itself plus stats that
// are useful to show off in a resume/demo (nodes expanded, cost, time).
struct AStarResult {
    bool found = false;
    std::vector<Point> path;          // start -> goal, inclusive
    std::vector<Point> visitedOrder;  // nodes popped from the open set, in order
    double totalCost = 0.0;
    long long nodesExpanded = 0;
    double timeMs = 0.0;
};

class AStarGrid {
public:
    explicit AStarGrid(const Grid& grid, bool allowDiagonal = true,
                        Heuristic heuristic = Heuristic::Diagonal);

    // Runs A* from start to goal. Returns a result with found=false if
    // no path exists (e.g. goal is walled off).
    AStarResult solve(const Point& start, const Point& goal) const;

private:
    const Grid& grid_;
    bool allowDiagonal_;
    Heuristic heuristic_;

    double heuristicCost(const Point& a, const Point& b) const;
    static std::vector<Point> reconstructPath(
        const std::unordered_map<Point, Point, PointHash>& cameFrom,
        const Point& start, const Point& current);
};

#endif // ASTAR_GRID_H
