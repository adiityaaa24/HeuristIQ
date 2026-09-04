// Unit tests for the grid representation and A* grid solver.
// Build & run via: make test   (see Makefile)

#include "mini_test.h"
#include "Grid.h"
#include "AStarGrid.h"
#include <cmath>

TEST_CASE(open_grid_diagonal_finds_straight_line_path) {
    Grid grid(5, 5);
    AStarGrid astar(grid, /*allowDiagonal=*/true, Heuristic::Diagonal);

    auto result = astar.solve({0, 0}, {4, 4});

    CHECK(result.found);
    CHECK(result.path.size() == 5); // 5 cells on a clean diagonal
    CHECK_NEAR(result.totalCost, 4 * std::sqrt(2.0), 1e-9);
}

TEST_CASE(wall_forces_a_detour_with_correct_cost) {
    Grid grid(5, 5);
    for (int r = 0; r < 4; ++r) grid.setObstacle({r, 2}); // wall, gap at row 4
    AStarGrid astar(grid, /*allowDiagonal=*/false, Heuristic::Manhattan);

    auto result = astar.solve({0, 0}, {0, 4});

    CHECK(result.found);
    // Must detour down to row 4 and back up: 4 + 4 + 4 = 12 steps.
    CHECK_NEAR(result.totalCost, 12.0, 1e-9);
}

TEST_CASE(fully_blocked_goal_reports_not_found) {
    Grid grid(5, 5);
    for (int r = 0; r < 5; ++r) grid.setObstacle({r, 2}); // solid wall, no gap
    AStarGrid astar(grid, false, Heuristic::Manhattan);

    auto result = astar.solve({0, 0}, {0, 4});

    CHECK(!result.found);
    CHECK(result.path.empty());
}

TEST_CASE(corner_clipping_through_diagonal_walls_is_prevented) {
    Grid grid(3, 3);
    grid.setObstacle({0, 1});
    grid.setObstacle({1, 0});
    AStarGrid astar(grid, /*allowDiagonal=*/true, Heuristic::Diagonal);

    // (0,0) -> (1,1) diagonally would cut through the corner formed by
    // the two walls above; that move must be rejected, and since it's
    // the only way out of the corner, no path should exist.
    auto result = astar.solve({0, 0}, {1, 1});

    CHECK(!result.found);
}

TEST_CASE(weighted_terrain_increases_path_cost) {
    Grid grid(3, 3);
    grid.setCost({0, 1}, 3);
    grid.setCost({0, 2}, 3);
    AStarGrid astar(grid, false, Heuristic::Manhattan);

    auto result = astar.solve({0, 0}, {0, 2});

    CHECK(result.found);
    CHECK_NEAR(result.totalCost, 6.0, 1e-9); // two cells at cost 3 each
}

TEST_CASE(astar_and_dijkstra_agree_on_optimal_cost) {
    Grid grid(6, 6);
    for (int r = 0; r < 5; ++r) grid.setObstacle({r, 2});

    AStarGrid astar(grid, true, Heuristic::Diagonal);
    AStarGrid dijkstra(grid, true, Heuristic::Zero);

    auto a = astar.solve({0, 0}, {5, 5});
    auto d = dijkstra.solve({0, 0}, {5, 5});

    CHECK(a.found && d.found);
    CHECK_NEAR(a.totalCost, d.totalCost, 1e-9);
    // The whole point of A*: same optimal answer, no more work than Dijkstra.
    CHECK(a.nodesExpanded <= d.nodesExpanded);
}

TEST_CASE(start_or_goal_on_an_obstacle_is_unsolvable) {
    Grid grid(4, 4);
    grid.setObstacle({2, 2});
    AStarGrid astar(grid, true, Heuristic::Diagonal);

    auto result = astar.solve({0, 0}, {2, 2}); // goal sits on a wall

    CHECK(!result.found);
}

TEST_CASE(start_equals_goal_returns_trivial_zero_cost_path) {
    Grid grid(4, 4);
    AStarGrid astar(grid, true, Heuristic::Diagonal);

    auto result = astar.solve({1, 1}, {1, 1});

    CHECK(result.found);
    CHECK(result.path.size() == 1);
    CHECK_NEAR(result.totalCost, 0.0, 1e-9);
}

int main() {
    std::cout << "Running HeuristIQ unit tests\n\n";
    return minitest::runAll();
}
