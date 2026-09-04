// HeuristIQ - Heuristic-Driven Pathfinding & Graph Search Engine (C++)
//
// Demonstrates:
//   1. A* search on a 2D grid (with obstacles, weighted terrain, diagonal
//      movement, and a choice of heuristics) via AStarGrid.
//   2. A* search on a generic weighted graph (adjacency list) via the
//      header-only GraphAStar<T> template - e.g. a small road network.
//   3. A* vs Dijkstra (heuristic = 0) comparison on the same grid, to show
//      how the heuristic reduces the number of nodes expanded.
//
// Build:  see README.md / Makefile / CMakeLists.txt
// Run:    ./heuristiq [path/to/map.txt]

#include "Grid.h"
#include "AStarGrid.h"
#include "GraphAStar.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <ctime>

static void printSeparator(const std::string& title) {
    std::cout << "\n=== " << title << " ===\n";
}

static Grid buildDemoGrid() {
    // 10x20 grid with a wall that has a single gap, plus a slow "swamp"
    // patch (cost 3) to show weighted-terrain behaviour.
    Grid grid(10, 20);
    for (int r = 0; r < 8; ++r) {
        grid.setObstacle({r, 10});
    }
    // gap in the wall at row 8
    for (int c = 2; c < 8; ++c) {
        for (int r = 4; r < 6; ++r) {
            grid.setCost({r, c}, 3); // swamp: costs 3x to enter
        }
    }
    return grid;
}

static void runGridDemo(const std::string& mapPath) {
    printSeparator("Grid Pathfinding (A*)");

    Grid grid = mapPath.empty() ? buildDemoGrid() : Grid::fromFile(mapPath);

    Point start{0, 0};
    Point goal{grid.rows() - 1, grid.cols() - 1};

    AStarGrid astar(grid, /*allowDiagonal=*/true, Heuristic::Diagonal);
    AStarResult result = astar.solve(start, goal);

    if (!result.found) {
        std::cout << "No path found between " << "(" << start.row << "," << start.col
                  << ") and (" << goal.row << "," << goal.col << ").\n";
        return;
    }

    grid.print(result.path, result.visitedOrder, &start, &goal);

    std::cout << "\nLegend: S=start  G=goal  *=path  :=explored  #=obstacle\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Path length (steps): " << result.path.size() - 1 << "\n";
    std::cout << "Path cost:           " << result.totalCost << "\n";
    std::cout << "Nodes expanded:      " << result.nodesExpanded << "\n";
    std::cout << "Time taken:          " << result.timeMs << " ms\n";
}

static void runAStarVsDijkstra() {
    printSeparator("A* vs Dijkstra (heuristic = 0) - nodes expanded comparison");

    Grid grid = buildDemoGrid();
    Point start{0, 0};
    Point goal{grid.rows() - 1, grid.cols() - 1};

    AStarGrid astar(grid, true, Heuristic::Diagonal);
    AStarGrid dijkstra(grid, true, Heuristic::Zero); // zero heuristic == Dijkstra

    AStarResult aResult = astar.solve(start, goal);
    AStarResult dResult = dijkstra.solve(start, goal);

    std::cout << "A*       -> nodes expanded: " << aResult.nodesExpanded
              << ", cost: " << aResult.totalCost << "\n";
    std::cout << "Dijkstra -> nodes expanded: " << dResult.nodesExpanded
              << ", cost: " << dResult.totalCost << "\n";
    std::cout << "Both should find the SAME optimal cost, but A* typically "
                 "expands fewer nodes because its heuristic guides the search.\n";
}

static void runGraphDemo() {
    printSeparator("Generic Graph A* (road-network style example)");

    // A small "city" graph. Node -> (straight-line distance to Home) is used
    // as the admissible heuristic, similar to real road-network routing.
    GraphAStar<std::string> cityGraph;
    cityGraph.addEdge("Home", "Market", 5.0);
    cityGraph.addEdge("Market", "Home", 5.0);
    cityGraph.addEdge("Home", "Park", 9.0);
    cityGraph.addEdge("Park", "Home", 9.0);
    cityGraph.addEdge("Market", "School", 3.0);
    cityGraph.addEdge("School", "Market", 3.0);
    cityGraph.addEdge("Market", "Park", 2.0);
    cityGraph.addEdge("Park", "Market", 2.0);
    cityGraph.addEdge("School", "Library", 6.0);
    cityGraph.addEdge("Library", "School", 6.0);
    cityGraph.addEdge("Park", "Library", 4.0);
    cityGraph.addEdge("Library", "Park", 4.0);

    // Straight-line (heuristic) distance estimates to "Library" - just for
    // demo purposes these are hand-picked plausible admissible values.
    std::unordered_map<std::string, double> heuristicToLibrary = {
        {"Home", 8.0}, {"Market", 5.0}, {"Park", 3.5}, {"School", 5.5}, {"Library", 0.0}
    };
    auto heuristic = [&](const std::string& node) {
        auto it = heuristicToLibrary.find(node);
        return it == heuristicToLibrary.end() ? 0.0 : it->second;
    };

    auto result = cityGraph.solve("Home", "Library", heuristic);

    if (!result.found) {
        std::cout << "No route found.\n";
        return;
    }

    std::cout << "Route: ";
    for (size_t i = 0; i < result.path.size(); ++i) {
        std::cout << result.path[i];
        if (i + 1 < result.path.size()) std::cout << " -> ";
    }
    std::cout << "\nTotal distance: " << result.totalCost
              << "\nNodes expanded: " << result.nodesExpanded << "\n";
}

int main(int argc, char* argv[]) {
    std::string mapPath = (argc > 1) ? argv[1] : "";

    std::cout << "HeuristIQ - Heuristic-Driven Pathfinding & Graph Search Engine\n";
    std::cout << "----------------------------------------------------------------\n";

    try {
        runGridDemo(mapPath);
        runAStarVsDijkstra();
        runGraphDemo();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    std::cout << "\nDone.\n";
    return 0;
}
