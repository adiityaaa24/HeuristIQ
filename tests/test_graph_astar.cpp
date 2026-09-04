// Unit tests for the generic, header-only GraphAStar<NodeId> template.
// Build & run via: make test   (see Makefile)

#include "mini_test.h"
#include "GraphAStar.h"
#include <string>

TEST_CASE(graph_astar_finds_shortest_route) {
    GraphAStar<std::string> g;
    g.addEdge("A", "B", 4.0);
    g.addEdge("B", "A", 4.0);
    g.addEdge("A", "C", 9.0);
    g.addEdge("C", "A", 9.0);
    g.addEdge("B", "C", 2.0);
    g.addEdge("C", "B", 2.0);

    auto zeroHeuristic = [](const std::string&) { return 0.0; }; // Dijkstra mode
    auto result = g.solve("A", "C", zeroHeuristic);

    CHECK(result.found);
    // Direct A->C costs 9; via B (A->B->C) costs 4+2=6, so that's optimal.
    CHECK_NEAR(result.totalCost, 6.0, 1e-9);
    CHECK(result.path.size() == 3); // A, B, C
}

TEST_CASE(graph_astar_reports_unreachable_node) {
    GraphAStar<std::string> g;
    g.addEdge("A", "B", 1.0);
    g.addEdge("B", "A", 1.0);
    // "Island" node with no edge connecting it to A/B.
    g.addEdge("Island", "Island", 0.0);

    auto zeroHeuristic = [](const std::string&) { return 0.0; };
    auto result = g.solve("A", "Island", zeroHeuristic);

    CHECK(!result.found);
}

TEST_CASE(graph_astar_works_with_integer_node_ids) {
    GraphAStar<int> g;
    g.addEdge(1, 2, 5.0);
    g.addEdge(2, 3, 5.0);
    g.addEdge(1, 3, 20.0);

    auto zeroHeuristic = [](int) { return 0.0; };
    auto result = g.solve(1, 3, zeroHeuristic);

    CHECK(result.found);
    CHECK_NEAR(result.totalCost, 10.0, 1e-9); // 1->2->3 beats the direct edge
}

int main() {
    std::cout << "Running GraphAStar unit tests\n\n";
    return minitest::runAll();
}
